#include <rtos.h>

#include <memory>
#include <vector>

#include <CommModule.hpp>
#include <CommPort.hpp>
#include <Decawave.hpp>
#include <SharedSPI.hpp>
#include <Assert.hpp>
#include <helper-funcs.hpp>
#include <logger.hpp>

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

void loopback_ack_pck(RTP::Packet p) {
    CommModule::Instance->send(std::move(p));
}

void legacy_rx_cb(RTP::Packet p) {
    if (p.payload.size()) {
        LOG(DEBUG,
            "Legacy rx successful!\r\n"
            "    Received: %u bytes\r\n",
            p.payload.size());
    } else {
        LOG(WARN, "Received empty packet on Legacy interface");
    }
}

void loopback_rx_cb(RTP::Packet p) {
    vector<uint16_t> duty_cycles;
    duty_cycles.assign(5, 100);
    for (size_t i = 0; i < duty_cycles.size(); ++i)
        duty_cycles.at(i) = 100 + 206 * i;

    if (p.payload.size()) {
        LOG(DEBUG,
            "Loopback rx successful!\r\n"
            "    Received: %u bytes",
            p.payload.size());
    } else {
        LOG(WARN, "Received empty packet on loopback interface");
    }
}

uint32_t loopback_tx_cb(const RTP::Packet* p) {
    if (p->payload.size()) {
        LOG(DEBUG,
            "Loopback tx successful!\r\n"
            "    Sent: %u bytes\r\n",
            p->payload.size());
    } else {
        LOG(WARN, "Sent empty packet on loopback interface");
    }

    CommModule::Instance->receive(*p);

    return COMM_SUCCESS;
}

void InitializeCommModule(SharedSPIDevice<>::SpiPtrT sharedSPI) {
// leds that flash if tx/rx have happened recently
#if defined(ENABLE_RX_TX_LEDS)
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
    auto commModule = CommModule::Instance;

    // TODO(justin): make this non-global
    // Create a new physical hardware communication link
    globalRadio =
        std::make_unique<Decawave>(sharedSPI, RJ_RADIO_nCS, RJ_RADIO_INT);

    // Open a socket for running tests across the link layer
    // The LINK port handlers are always active, regardless of whether or not a
    // working radio is connected.
    commModule->setRxHandler(&loopback_rx_cb, RTP::PortType::LINK);
    commModule->setTxHandler(&loopback_tx_cb, RTP::PortType::LINK);

    /*
     * Ports are always displayed in ascending (lowest -> highest) order
     * according to its port number when using the console.
     */
    if (globalRadio->isConnected() == true) {
        LOG(OK, "Radio interface ready!");

        // Legacy port
        commModule->setTxHandler(globalRadio.get(), &CommLink::sendPacket,
                                 RTP::PortType::LEGACY);
        commModule->setRxHandler(&legacy_rx_cb, RTP::PortType::LEGACY);

        LOG(INFO, "%u sockets opened", commModule->numOpenSockets());

        // Wait until the threads with the commModule are all started up
        // and ready
        while (!commModule->isReady()) {
            Thread::wait(50);
        }
    } else {
        LOG(SEVERE, "No radio interface found!");
    }
}
