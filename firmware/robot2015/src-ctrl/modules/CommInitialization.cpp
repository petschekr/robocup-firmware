#include <rtos.h>

#include <memory>
#include <vector>

// #include <CC1201Radio.hpp>
#include <CommModule.hpp>
#include <CommPort.hpp>
#include <assert.hpp>
#include <helper-funcs.hpp>
#include <logger.hpp>
#include "Decawave.hpp"

#include "TimeoutLED.hpp"
#include "fpga.hpp"
#include "io-expander.hpp"
#include "robot-devices.hpp"
#include "task-signals.hpp"

using namespace std;

// Comment/uncomment this line to control whether or not the rx/tx leds are used
// #define ENABLE_RX_TX_LEDS

/*
 * Information about the radio protocol can be found at:
 * https://www.overleaf.com/2187548nsfdps
 */

void loopback_ack_pck(rtp::packet p) {
    CommModule::Instance->send(std::move(p));
}

void legacy_rx_cb(rtp::packet p) {
    if (p.payload.size()) {
        LOG(OK,
            "Legacy rx successful!\r\n"
            "    Received: %u bytes\r\n",
            p.payload.size());
    } else {
        LOG(WARN, "Received empty packet on Legacy interface");
    }
}

void loopback_rx_cb(rtp::packet p) {
    vector<uint16_t> duty_cycles;
    duty_cycles.assign(5, 100);
    for (size_t i = 0; i < duty_cycles.size(); ++i)
        duty_cycles.at(i) = 100 + 206 * i;

    if (p.payload.size()) {
        LOG(OK,
            "Loopback rx successful!\r\n"
            "    Received: %u bytes",
            p.payload.size());
    } else {
        LOG(WARN, "Received empty packet on loopback interface");
    }
}

int32_t loopback_tx_cb(const rtp::packet* p) {
    if (p->payload.size()) {
        LOG(OK,
            "Loopback tx successful!\r\n"
            "    Sent: %u bytes\r\n",
            p->payload.size());
    } else {
        LOG(WARN, "Sent empty packet on loopback interface");
    }

    CommModule::Instance->receive(*p);

    return COMM_SUCCESS;
}

void InitializeCommModule(shared_ptr<SharedSPI> sharedSPI) {
// leds that flash if tx/rx have happened recently
#ifdef ENABLE_RX_TX_LEDS
    auto rxTimeoutLED = make_shared<FlashingTimeoutLED>(
        DigitalOut(RJ_RX_LED, OpenDrain), 160, 400);
    auto txTimeoutLED = make_shared<FlashingTimeoutLED>(
        DigitalOut(RJ_TX_LED, OpenDrain), 160, 400);
#else
    auto rxTimeoutLED = nullptr;
    auto txTimeoutLED = nullptr;
#endif

    // Startup the CommModule interface
    CommModule::Instance = make_shared<CommModule>(rxTimeoutLED, txTimeoutLED);
    shared_ptr<CommModule> commModule = CommModule::Instance;

    // TODO(justin): make this non-global
    // Create a new physical hardware communication link
    global_radio = new Decawave(sharedSPI, RJ_RADIO_nCS, RJ_RADIO_INT);

    // Open a socket for running tests across the link layer
    // The LINK port handlers are always active, regardless of whether or not a
    // working radio is connected.
    commModule->setRxHandler(&loopback_rx_cb, rtp::Port::LINK);
    commModule->setTxHandler(&loopback_tx_cb, rtp::Port::LINK);

    /*
     * Ports are always displayed in ascending (lowest -> highest) order
     * according to its port number when using the console.
     */
    if (global_radio->isConnected() == true) {
        // LOG(INIT, "Radio interface ready on %3.2fMHz!",
        // global_radio->freq());
        LOG(INIT, "Radio interface ready");

        // Legacy port
        commModule->setTxHandler((CommLink*)global_radio, &CommLink::sendPacket,
                                 rtp::Port::LEGACY);
        commModule->setRxHandler(&legacy_rx_cb, rtp::Port::LEGACY);

        LOG(INIT, "%u sockets opened", commModule->numOpenSockets());

        // Wait until the threads with the commModule are all started up
        // and ready
        while (!commModule->isReady()) {
            Thread::wait(50);
        }
    } else {
        LOG(FATAL, "No radio interface found!\r\n");
    }
}
