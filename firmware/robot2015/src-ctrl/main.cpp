// ** DON'T INCLUDE <iostream>! THINGS WILL BREAK! **
#include <array>
#include <ctime>
#include <string>

#include <rtos.h>

#include <assert.hpp>
#include <helper-funcs.hpp>
#include <logger.hpp>
#include <watchdog.hpp>

#include "BallSense.hpp"
#include "CC1201.cpp"
#include "KickerBoard.hpp"
#include "RadioProtocol.hpp"
#include "RotarySelector.hpp"
#include "RtosTimerHelper.hpp"
#include "SharedSPI.hpp"
#include "commands.hpp"
#include "fpga.hpp"
#include "io-expander.hpp"
#include "io-expander.hpp"
#include "neostrip.hpp"
#include "robot-devices.hpp"
#include "task-signals.hpp"

#define RJ_ENABLE_ROBOT_CONSOLE

using namespace std;

void Task_Controller(void const* args);

/**
 * @brief Sets the hardware configurations for the status LEDs & places
 * into the given state
 *
 * @param[in] state The next state of the LEDs
 */
void statusLights(bool state) {
    DigitalOut init_leds[] = {
        {RJ_BALL_LED}, {RJ_RX_LED}, {RJ_TX_LED}, {RJ_RDY_LED}};
    // the state is inverted because the leds are wired active-low
    for (DigitalOut& led : init_leds) led = !state;
}

/**
 * The entry point of the system where each submodule's thread is started.
 */
int main() {
    // Store the thread's ID
    const osThreadId mainID = Thread::gettid();
    ASSERT(mainID != nullptr);

    // clear any extraneous rx serial bytes
    Serial s(RJ_SERIAL_RXTX);
    while (s.readable()) s.getc();

    // set baud rate to higher value than the default for faster terminal
    s.baud(57600);

    // Turn on some startup LEDs to show they're working, they are turned off
    // before we hit the while loop
    statusLights(true);

    // Set the default logging configurations
    isLogging = RJ_LOGGING_EN;
    rjLogLevel = INIT;

    /* Always send out an empty line at startup for keeping the console
     * clean on after a 'reboot' command is called;
     */
    if (isLogging) {
        // reset the console's default settings and enable the cursor
        printf("\033[m");
        fflush(stdout);
    }

    // Setup the interrupt priorities before launching each subsystem's task
    // thread.
    setISRPriorities();

    // Initialize and start ball sensor
    BallSense ballSense(RJ_BALL_EMIT, RJ_BALL_DETECTOR);
    ballSense.start(10);
    DigitalOut ballSenseStatusLED(RJ_BALL_LED, 1);

    // Force off since the neopixel's hardware is stateless from previous
    // settings
    NeoStrip rgbLED(RJ_NEOPIXEL, 2);
    rgbLED.clear();

    // Set the RGB LEDs to a medium blue while the threads are started up
    float defaultBrightness = 0.02f;
    rgbLED.brightness(3 * defaultBrightness);
    rgbLED.setPixel(0, NeoColorBlue);
    rgbLED.setPixel(1, NeoColorBlue);
    rgbLED.write();

    // Flip off the startup LEDs after a timeout period
    RtosTimerHelper init_leds_off([]() { statusLights(false); }, osTimerOnce);
    init_leds_off.start(RJ_STARTUP_LED_TIMEOUT_MS);

    /// A shared spi bus used for the fpga and cc1201 radio
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(8, 0);  // 8 bits per transfer

    // Initialize and configure the fpga with the given bitfile
    FPGA::Instance = new FPGA(sharedSPI, RJ_FPGA_nCS, RJ_FPGA_INIT_B,
                              RJ_FPGA_PROG_B, RJ_FPGA_DONE);
    bool fpgaReady = FPGA::Instance->configure("/local/rj-fpga.nib");

    if (fpgaReady) {
        rgbLED.brightness(3 * defaultBrightness);
        rgbLED.setPixel(1, NeoColorGreen);

        LOG(INIT, "FPGA Configuration Successful!");

    } else {
        rgbLED.brightness(4 * defaultBrightness);
        rgbLED.setPixel(1, NeoColorOrange);

        LOG(FATAL, "FPGA Configuration Failed!");
    }
    rgbLED.write();

    DigitalOut rdy_led(RJ_RDY_LED, !fpgaReady);

    // Initialize kicker board
    // TODO: clarify between kicker nCs and nReset
    KickerBoard kickerBoard(sharedSPI, RJ_KICKER_nCS, RJ_KICKER_nRESET,
                            "/local/rj-kickr.nib");
    bool kickerReady = kickerBoard.flash(true, true);

    // Init IO Expander and turn all LEDs on.  The first parameter to config()
    // sets the first 8 lines to input and the last 8 to output.  The pullup
    // resistors and polarity swap are enabled for the 4 rotary selector lines.
    MCP23017 ioExpander(RJ_I2C_SDA, RJ_I2C_SCL, RJ_IO_EXPANDER_I2C_ADDRESS);
    ioExpander.config(0x00FF, 0x00ff, 0x00ff);
    ioExpander.writeMask((uint16_t)~IOExpanderErrorLEDMask,
                         IOExpanderErrorLEDMask);

    // DIP Switch 1 controls the radio channel.
    uint8_t currentRadioChannel = 0;
    IOExpanderDigitalInOut radioChannelSwitch(&ioExpander, RJ_DIP_SWITCH_1,
                                              MCP23017::DIR_INPUT);

    // rotary selector for shell id
    RotarySelector<IOExpanderDigitalInOut> rotarySelector(
        {IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT0,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT1,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT2,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT3,
                                MCP23017::DIR_INPUT)});
    // this value is continuously updated in the main loop
    uint8_t robotShellID = rotarySelector.read();

    // Startup the 3 separate threads, being sure that we wait for it
    // to signal back to us that we can startup the next thread. Not doing
    // so results in weird wierd things that are really hard to debug. Even
    // though this is multi-threaded code, that dosen't mean it's
    // a multi-core system.

    // Start the thread task for the on-board control loop
    Thread controller_task(Task_Controller, mainID, osPriorityHigh,
                           DEFAULT_STACK_SIZE / 2);
    Thread::signal_wait(MAIN_TASK_CONTINUE, osWaitForever);

#ifdef RJ_ENABLE_ROBOT_CONSOLE
    // Start the thread task for the serial console
    Thread console_task(Task_SerialConsole, mainID, osPriorityBelowNormal);
    Thread::signal_wait(MAIN_TASK_CONTINUE, osWaitForever);
#endif

    // Initialize the CommModule and CC1201 radio
    InitializeCommModule(sharedSPI);

    // Make sure all of the motors are enabled
    motors_Init();

    // setup analog in on battery sense pin
    // the value is updated in the main loop below
    AnalogIn batt(RJ_BATT_SENSE);
    uint8_t battVoltage = 0;

    // Setup radio protocol handling
    RadioProtocol radioProtocol(CommModule::Instance, global_radio);
    radioProtocol.setUID(robotShellID);
    radioProtocol.start();
    radioProtocol.rxCallback = [&](const rtp::ControlMessage* msg) {
        rtp::RobotStatusMessage reply;
        reply.uid = robotShellID;
        reply.battVoltage = battVoltage;
        reply.ballSenseStatus = ballSense.have_ball() ? 1 : 0;

        vector<uint8_t> replyBuf;
        rtp::SerializeToVector(reply, &replyBuf);
        return replyBuf;
    };


    kickerBoard.charge();
    LOG(INIT, "Starged charging kicker board");
    uint8_t kickerVoltage = 0;

    // Set the watdog timer's initial config
    Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);

    // Release each thread into its operations in a structured manner
    controller_task.signal_set(SUB_TASK_CONTINUE);
#ifdef RJ_ENABLE_ROBOT_CONSOLE
    console_task.signal_set(SUB_TASK_CONTINUE);
#endif

    osStatus tState = osThreadSetPriority(mainID, osPriorityNormal);
    ASSERT(tState == osOK);

    unsigned int ll = 0;
    uint16_t errorBitmask = 0;
    if (!fpgaReady) {
        // assume all motors have errors if FPGA does not work
        errorBitmask |= (1 << RJ_ERR_LED_M1);
        errorBitmask |= (1 << RJ_ERR_LED_M2);
        errorBitmask |= (1 << RJ_ERR_LED_M3);
        errorBitmask |= (1 << RJ_ERR_LED_M4);
        errorBitmask |= (1 << RJ_ERR_LED_DRIB);
    }

    while (true) {
        // make sure we can always reach back to main by
        // renewing the watchdog timer periodicly
        Watchdog::Renew();

        // periodically reset the console text's format
        ll++;
        if ((ll % 8) == 0) {
            printf("\033[m");
            fflush(stdout);
        }

        Thread::wait(RJ_WATCHDOG_TIMER_VALUE * 250);

        // the value is inverted because this led is wired active-low
        ballSenseStatusLED = !ballSense.have_ball();

        // Pack errors into bitmask
        errorBitmask |= (!global_radio || !global_radio->isConnected())
                        << RJ_ERR_LED_RADIO;

        motors_refresh();

        // add motor errors to bitmask
        static const auto motorErrLedMapping = {
            make_pair(0, RJ_ERR_LED_M1), make_pair(1, RJ_ERR_LED_M2),
            make_pair(2, RJ_ERR_LED_M3), make_pair(3, RJ_ERR_LED_M4),
            make_pair(4, RJ_ERR_LED_DRIB)};

        for (auto& pair : motorErrLedMapping) {
            const motorErr_t& status = global_motors[pair.first].status;
            // clear the bit
            errorBitmask &= ~(1 << pair.second);
            // set the bit to whatever hasError is set to
            errorBitmask |= (status.hasError << pair.second);
        }

        // get the battery voltage
        battVoltage = (batt.read_u16() >> 8);

        // get kicker voltage
        kickerVoltage = kickerBoard.read_voltage();
        LOG(INIT, "Kicker voltage: %u", kickerVoltage);

        // update shell id
        robotShellID = rotarySelector.read();
        radioProtocol.setUID(robotShellID);

        // update radio channel
        uint8_t newRadioChannel = radioChannelSwitch.read();
        if (newRadioChannel != currentRadioChannel) {
            global_radio->setChannel(newRadioChannel);
            currentRadioChannel = newRadioChannel;
            LOG(INIT, "Changed radio channel to %u", newRadioChannel);
        }

        // Set error-indicating leds on the control board
        ioExpander.writeMask(~errorBitmask, IOExpanderErrorLEDMask);

        if (errorBitmask || !fpgaReady) {
            // orange - error
            rgbLED.brightness(6 * defaultBrightness);
            rgbLED.setPixel(0, NeoColorOrange);
        } else {
            // no errors, yay!
            rgbLED.brightness(3 * defaultBrightness);
            rgbLED.setPixel(0, NeoColorGreen);
        }
        rgbLED.write();
    }
}

#define _EXTERN extern "C"

_EXTERN void HardFault_Handler() {
    __asm volatile(
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, hard_fault_handler_2_const                        \n"
        " bx r2                                                     \n"
        " hard_fault_handler_2_const: .word HARD_FAULT_HANDLER    	\n");
}

_EXTERN void HARD_FAULT_HANDLER(uint32_t* stackAddr) {
    /* These are volatile to try and prevent the compiler/linker optimising them
     * away as the variables never actually get used.  If the debugger won't
     * show the values of the variables, make them global my moving their
     * declaration outside of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;  /* Link register. */
    volatile uint32_t pc;  /* Program counter. */
    volatile uint32_t psr; /* Program status register. */

    r0 = stackAddr[0];
    r1 = stackAddr[1];
    r2 = stackAddr[2];
    r3 = stackAddr[3];
    r12 = stackAddr[4];
    lr = stackAddr[5];
    pc = stackAddr[6];
    psr = stackAddr[7];

    LOG(FATAL,
        "\r\n"
        "================================\r\n"
        "========== HARD FAULT ==========\r\n"
        "\r\n"
        "  MSP:\t0x%08X\r\n"
        "  HFSR:\t0x%08X\r\n"
        "  CFSR:\t0x%08X\r\n"
        "\r\n"
        "  r0:\t0x%08X\r\n"
        "  r1:\t0x%08X\r\n"
        "  r2:\t0x%08X\r\n"
        "  r3:\t0x%08X\r\n"
        "  r12:\t0x%08X\r\n"
        "  lr:\t0x%08X\r\n"
        "  pc:\t0x%08X\r\n"
        "  psr:\t0x%08X\r\n"
        "\r\n"
        "========== HARD FAULT ==========\r\n"
        "================================",
        __get_MSP, SCB->HFSR, SCB->CFSR, r0, r1, r2, r3, r12, lr, pc, psr);

    // do nothing so everything remains unchanged for debugging
    while (true) {
    }
}

_EXTERN void NMI_Handler() { std::printf("NMI Fault!\n"); }

_EXTERN void MemManage_Handler() { std::printf("MemManage Fault!\n"); }

_EXTERN void BusFault_Handler() { std::printf("BusFault Fault!\n"); }

_EXTERN void UsageFault_Handler() { std::printf("UsageFault Fault!\n"); }