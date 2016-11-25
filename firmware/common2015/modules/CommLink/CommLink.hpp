#pragma once

#include <mbed.h>
#include <rtos.h>

#include "CommModule.hpp"
#include "RTP.hpp"
#include "SharedSPI.hpp"
#include "helper-funcs.hpp"
#include "rj-macros.hpp"
#include "rtos-mgmt/mail-helper.hpp"

#define FOREACH_COMM_ERR(ERR) \
    ERR(COMM_SUCCESS)         \
    ERR(COMM_FAILURE)         \
    ERR(COMM_DEV_BUF_ERR)     \
    ERR(COMM_FUNC_BUF_ERR)    \
    ERR(COMM_FALSE_TRIG)      \
    ERR(COMM_NO_DATA)

/**
 * CommLink Error Levels.
 */
enum { FOREACH_COMM_ERR(GENERATE_ENUM) };

/**
 * CommLink Class used as the hal (hardware abstraction layer) module for
 * interfacing communication links to the higher-level firmware
 */
class CommLink : public SharedSPIDevice<> {
public:
    // Type aliases
    using BufferT = std::vector<uint8_t>;
    using BufferPtrT = BufferT*;
    using ConstBufferPtrT = const BufferT*;

    // Class constants for data queues
    static constexpr size_t RX_QUEUE_SIZE = 2;

    /// Constructor
    CommLink(spiPtr_t spiBus, PinName nCs = NC, PinName intPin = NC);

    /// Virtual deconstructor
    /// Kills any threads and frees the allocated stack.
    virtual ~CommLink() {}

    // The pure virtual methods for making CommLink an abstract class
    /// Perform a soft reset for a communication link's hardware device
    virtual void reset() = 0;

    /// Perform tests to determine if the hardware is able to properly function
    virtual int32_t selfTest() = 0;

    /// Determine if communication can occur with another device
    virtual bool isConnected() const = 0;

    /// Send & Receive through the rtp structure
    virtual int32_t sendPacket(const RTP::Packet* pkt) = 0;

protected:
    static constexpr size_t SIGNAL_START = (1 << 1);
    static constexpr size_t SIGNAL_RX = (1 << 1);

    InterruptIn m_intIn;

    /**
     * @brief Read data from the radio's RX buffer
     *
     * @param buf The buffer to write data into
     *
     * @return A vector of received bytes returned with std::move
     */
    virtual BufferT getData() = 0;

    /// Interrupt Service Routine
    void ISR() { m_rxThread.signal_set(SIGNAL_RX); }

    /// Called by the derived class to begin thread operations
    void ready() { m_rxThread.signal_set(SIGNAL_START); }

    template <typename T>
    constexpr static T twos_compliment(T val) {
        return ~val + 1;
    }

private:
    // DEFAULT_STACK_SIZE defined in rtos library
    static constexpr size_t STACK_SIZE = DEFAULT_STACK_SIZE / 2;

    Thread m_rxThread;

    // The working thread for handling RX data queue operations
    void rxThread();

    inline static void rxThreadHelper(const void* linkInst) {
        auto link = reinterpret_cast<CommLink*>(
            const_cast<void*>(linkInst));  // dangerous
        link->rxThread();
    }
};
