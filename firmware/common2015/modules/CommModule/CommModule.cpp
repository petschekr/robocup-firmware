#include "CommModule.hpp"
#include "CommPort.hpp"
#include "assert.hpp"
#include "helper-funcs.hpp"
#include "logger.hpp"

#include <ctime>

using namespace std;

constexpr auto COMM_MODULE_SIGNAL_START_THREAD = (1 << 0);

std::shared_ptr<CommModule> CommModule::Instance;

CommModule::CommModule(std::shared_ptr<FlashingTimeoutLED> rxTimeoutLED,
                       std::shared_ptr<FlashingTimeoutLED> txTimeoutLED)
    : m_rxThread(&CommModule::rxThreadHelper, this, osPriorityAboveNormal,
                 DEFAULT_STACK_SIZE / 2),
      m_txThread(&CommModule::txThreadHelper, this, osPriorityHigh,
                 DEFAULT_STACK_SIZE / 2),
      m_rxTimeoutLED(rxTimeoutLED),
      m_txTimeoutLED(txTimeoutLED) {
    // Create the data queues.
    m_txQueue = osMailCreate(m_txQueueHelper.def(), nullptr);
    m_rxQueue = osMailCreate(m_rxQueueHelper.def(), nullptr);
}

void CommModule::rxThreadHelper(void const* moduleInst) {
    auto module = reinterpret_cast<CommModule*>(const_cast<void*>(moduleInst));
    module->rxThread();
}
void CommModule::txThreadHelper(void const* moduleInst) {
    auto module = reinterpret_cast<CommModule*>(const_cast<void*>(moduleInst));
    module->txThread();
}

void CommModule::txThread() {
    // Only continue once we know there's 1+ hardware link(s) available
    Thread::signal_wait(COMM_MODULE_SIGNAL_START_THREAD);

    // Store our priority so we know what to reset it to if ever needed
    const osPriority threadPriority = m_txThread.get_priority();

    LOG(INIT,
        "TX communication module ready!\r\n    Thread ID: %u, Priority: %d",
        reinterpret_cast<P_TCB>(m_rxThread.gettid())->task_id, threadPriority);

    // Signal to the RX thread that it can begin
    m_rxThread.signal_set(COMM_MODULE_SIGNAL_START_THREAD);

    while (true) {
        // Wait until new data is placed in the RX queue
        auto evt = osMailGet(m_txQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // Get a pointer to the packet's memory location
            auto p = static_cast<RTP::Packet*>(evt.value.p);

            // Bump up the thread's priority
            auto tState = m_txThread.set_priority(osPriorityRealtime);
            ASSERT(tState == osOK);

            // cache the dereference to the header
            const auto& header = p->header;

            // this renews a countdown for turning off the strobing thread
            if (header.address != RTP::LOOPBACK_ADDRESS && m_txTimeoutLED) {
                m_txTimeoutLED->renew();
            }

            // grab the port number
            const auto portNum = header.port;

            // Call the user callback function
            if (m_ports.find(portNum) != m_ports.end()) {
                // only seek the port's reference once
                auto& port = m_ports[portNum];

                if (port.txCallback() != nullptr) {
                    port.txCallback()(p);
                    port.m_txCount++;

                    LOG(INF2, "Transmission:\r\n    Port:\t%u\r\n", portNum);
                }
            }

            // Release the allocated memory once data is sent
            osMailFree(m_txQueue, p);

            tState = m_txThread.set_priority(threadPriority);
            ASSERT(tState == osOK);
        }
    }
}

void CommModule::rxThread() {
    // Only continue once we know there's 1+ hardware link(s) available
    Thread::signal_wait(COMM_MODULE_SIGNAL_START_THREAD);

    // set this true immediately after we are released execution
    m_isReady = true;

    // Store our priority so we know what to reset it to if ever needed
    const auto threadPriority = m_rxThread.get_priority();

    LOG(INIT,
        "RX communication module ready!\r\n    Thread ID: %u, Priority: %d",
        reinterpret_cast<P_TCB>(m_rxThread.gettid())->task_id, threadPriority);

    while (true) {
        // Wait until new data is placed in the RX queue
        auto evt = osMailGet(m_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // get a pointer to where the data is stored
            auto p = static_cast<RTP::Packet*>(evt.value.p);

            // Bump up the thread's priority
            auto tState = m_rxThread.set_priority(osPriorityRealtime);
            ASSERT(tState == osOK);

            // cache the dereference to the header
            const auto& header = p->header;

            // this renews a countdown for turning off the strobing thread
            if (header.address != RTP::LOOPBACK_ADDRESS && m_rxTimeoutLED) {
                m_rxTimeoutLED->renew();
            }

            // grab the port number
            const auto portNum = header.port;

            // Call the user callback function
            if (m_ports.find(portNum) != m_ports.end()) {
                // only seek the port's reference once
                auto& port = m_ports[portNum];

                if (port.rxCallback() != nullptr) {
                    port.rxCallback()(std::move(*p));
                    port.m_rxCount++;

                    LOG(INF2, "Transmission:\r\n    Port:\t%u\r\n", portNum);
                    LOG(INF2, "Reception:\r\n    Port:\t%u\r\n", portNum);
                }
            }

            // free memory allocated for mail
            osMailFree(m_rxQueue, p);

            tState = m_rxThread.set_priority(threadPriority);
            ASSERT(tState == osOK);
        }
    }
}

void CommModule::setRxHandler(RxCallbackT callback, uint8_t portNbr) {
    m_ports[portNbr].rxCallback() = std::bind(callback, std::placeholders::_1);
    ready();
}

void CommModule::setTxHandler(TxCallbackT callback, uint8_t portNbr) {
    m_ports[portNbr].txCallback() = std::bind(callback, std::placeholders::_1);
    ready();
}

void CommModule::ready() {
    if (m_isReady) return;

    // Start running the TX thread - that then starts the RX once
    m_txThread.signal_set(COMM_MODULE_SIGNAL_START_THREAD);
}

void CommModule::send(RTP::Packet packet) {
    // Check to make sure a socket for the port exists
    if (m_ports.find(packet.header.port) != m_ports.end() &&
        m_ports[packet.header.port].txCallback() != nullptr) {
        // Allocate a block of memory for the data.
        auto p =
            static_cast<RTP::Packet*>(osMailAlloc(m_txQueue, osWaitForever));
        if (!p) {
            LOG(FATAL, "Unable to allocate packet onto mail queue");
            return;
        }

        // Copy the contents into the allocated memory block
        *p = std::move(packet);

        // Place the passed packet into the txQueue.
        osMailPut(m_txQueue, p);

    } else {
        LOG(WARN,
            "Failed to send %u byte packet: There is no open transmitting "
            "socket for port %u",
            packet.payload.size(), packet.header.port);
    }
}

void CommModule::receive(RTP::Packet packet) {
    // Check to make sure a socket for the port exists
    if (m_ports.find(packet.header.port) != m_ports.end() &&
        m_ports[packet.header.port].rxCallback() != nullptr) {
        // Allocate a block of memory for the data.
        auto p =
            static_cast<RTP::Packet*>(osMailAlloc(m_rxQueue, osWaitForever));
        if (!p) {
            LOG(FATAL, "Unable to allocate packet onto mail queue");
            return;
        }

        // Copy the contents into the allocated memory block
        *p = std::move(packet);

        // Place the passed packet into the rxQueue.
        osMailPut(m_rxQueue, p);
    } else {
        LOG(WARN,
            "Failed to receive %u byte packet: There is no open receiving "
            "socket for port %u",
            packet.payload.size(), packet.header.port);
    }
}

unsigned int CommModule::numRxPackets() const {
    auto count = 0;
    for (auto& kvpair : m_ports) count += kvpair.second.m_rxCount;
    return count;
}

unsigned int CommModule::numTxPackets() const {
    auto count = 0;
    for (auto& kvpair : m_ports) count += kvpair.second.m_txCount;
    return count;
}

void CommModule::printInfo() const {
    printf("PORT\t\tIN\tOUT\tRX CBCK\t\tTX CBCK\r\n");

    for (const auto& kvpair : m_ports) {
        const PortT& p = kvpair.second;
        printf("%d\t\t%u\t%u\t%s\t\t%s\r\n", kvpair.first, p.m_rxCount,
               p.m_txCount, p.rxCallback() ? "YES" : "NO",
               p.txCallback() ? "YES" : "NO");
    }

    printf(
        "==========================\r\n"
        "Total:\t\t%u\t%u\r\n",
        numRxPackets(), numTxPackets());

    Console::Instance()->Flush();
}

void CommModule::resetCount(unsigned int portNbr) {
    m_ports[portNbr].resetPacketCount();
}

int CommModule::numOpenSockets() const {
    auto count = 0;
    for (const auto& kvpair : m_ports) {
        if (kvpair.second.rxCallback() != nullptr ||
            kvpair.second.txCallback() != nullptr)
            count++;
    }
    return count;
}
