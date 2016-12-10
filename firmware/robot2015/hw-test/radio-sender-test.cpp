/**
 * This program repeatedly sends a control packet to robot 1.  It prints a '-->'
 * to the console for every packet sent and a '<--' for every packet received.
 * This program was created to test the cc1201 configuration/driver in order to
 * ensure that everything works and to tune the register settings.  It is meant
 * to be used along with the radio-receiver-test program.
 */

#include <cmsis_os.h>
#include <mbed.h>
#include <memory>

#include "CC1201Radio.hpp"
#include "SharedSPI.hpp"
#include "logger.hpp"
#include "pins-ctrl-2015.hpp"

using namespace std;

// how often to send a packet (in seconds)
const float TRANSMIT_INTERVAL = 1.0f / 100.0f;

bool initRadio() {
    // setup SPI bus
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(8, 0);  // 8 bits per transfer

    // RX/TX leds
    auto rxTimeoutLED = make_shared<FlashingTimeoutLED>(LED1);
    auto txTimeoutLED = make_shared<FlashingTimeoutLED>(LED2);

    // Startup the CommModule interface
    CommModule::Instance = make_shared<CommModule>(rxTimeoutLED, txTimeoutLED);

    // Construct an object pointer for the radio
    constexpr auto settingsSize = sizeof(preferredSettings) / sizeof(registerSetting_t);
    global_radio = std::make_unique<CC1201>(sharedSPI, RJ_RADIO_nCS, RJ_RADIO_INT, preferredSettings, settingsSize);

    return global_radio->isConnected();
}

void radioRxHandler(RTP::Packet pkt) {
    static int rxCount = 0;
    ++rxCount;
    printf("<-- %d\r\n", rxCount);
}

int main() {
    // set baud rate to higher value than the default for faster terminal
    Serial s(RJ_SERIAL_RXTX);
    s.baud(57600);

    // Set the default logging configurations
    isLogging = RJ_LOGGING_EN;
    rjLogLevel = INIT;

    printf("****************************************\r\n");
    LOG(INIT, "Radio test sender starting...");

    if (initRadio()) {
        LOG(INIT, "Radio interface ready on %3.2fMHz!", global_radio->freq());

        // register handlers for any ports we might use
        for (RTP::Port port :
             {RTP::Port::CONTROL, RTP::Port::PING, RTP::Port::LEGACY}) {
            CommModule::Instance->setRxHandler(&radioRxHandler, port);
            CommModule::Instance->setTxHandler((CommLink*)global_radio,
                                               &CommLink::sendPacket, port);
        }
    } else {
        LOG(FATAL, "No radio interface found!");
    }

    DigitalOut senderIndicator(LED3, 1);
    DigitalOut radioStatusLed(LED4, global_radio->isConnected());

    // send packets every @TRANSMIT_INTERVAL forever
    while (true) {
        static int txCount = 0;

        RTP::Packet pkt;
        pkt.header.port = RTP::Port::CONTROL;
        pkt.header.address = RTP::BROADCAST_ADDRESS;

        // create control message and add it to the packet payload
        RTP::ControlMessage msg;
        msg.uid = 1;  // address message to robot 1
        msg.bodyX = 2;
        msg.bodyY = 3;
        msg.bodyW = 4;
        RTP::SerializeToVector(msg, &pkt.payload);

        // transmit!
        CommModule::Instance->send(std::move(pkt));
        txCount++;
        printf("--> %d\r\n", txCount);

        Thread::wait(TRANSMIT_INTERVAL * 1e3);
    }
}
