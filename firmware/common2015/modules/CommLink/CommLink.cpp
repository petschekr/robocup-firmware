#include "CommLink.hpp"

#include "assert.hpp"
#include "logger.hpp"

namespace {
// DEFAULT_STACK_SIZE defined in rtos library
constexpr auto STACK_SIZE = DEFAULT_STACK_SIZE / 2;
constexpr auto RX_PRIORITY = osPriorityNormal;
}

std::unique_ptr<CommLink> global_radio = nullptr;

static constexpr const char* COMM_ERR_STRING[] = {
    FOREACH_COMM_ERR(GENERATE_STRING)};

CommLink::CommLink(SpiPtrT sharedSPI, PinName nCs, PinName intPin)
    : SharedSPIDevice(sharedSPI, nCs, true),
      m_intIn(intPin),
      m_rxThread(&CommLink::rxThreadHelper, this, RX_PRIORITY, STACK_SIZE) {
    setSPIFrequency(5'000'000);
    m_intIn.mode(PullDown);
    ready();
}

// Task operations for placing received data into the received data queue
void CommLink::rxThread() {
    // Store our priority so we know what to reset it to if ever needed
    const auto threadPriority = m_rxThread.get_priority();
    ASSERT(threadPriority != osPriorityError);

    // Set the function to call on an interrupt trigger
    m_intIn.rise(this, &CommLink::ISR);

    // Only continue past this point once the hardware link is initialized
    Thread::signal_wait(SIGNAL_START);

    LOG(INIT, "RX communication link ready!\r\n    Thread ID: %u, Priority: %d",
        reinterpret_cast<P_TCB>(m_rxThread.gettid())->task_id, threadPriority);

    while (true) {
        // Thread::yield();

        // Wait until new data has arrived
        Thread::signal_wait(SIGNAL_RX);

        LOG(INF3, "RX interrupt triggered");

        // Get the received data from the external chip
        auto response = getData();

        // Thread::yield();

        if (!response.empty()) {
            // Write the data to the CommModule object's rxQueue
            RTP::Packet p(response);
            CommModule::Instance->receive(std::move(p));
        }
    }

    ASSERT(!"Execution is at an unreachable line!");
}
