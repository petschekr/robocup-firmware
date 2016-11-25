#pragma once

#include <mbed.h>
#include <rtos.h>

#include "CommPort.hpp"
#include "Console.hpp"
#include "RTP.hpp"
#include "TimeoutLED.hpp"
#include "helper-funcs.hpp"
#include "rtos-mgmt/mail-helper.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

/**
 * @brief A high-level firmware class for packet handling & routing
 *
 * The CommModule class provides the packet management routing
 * by distributing incoming packets to the correct area of the
 * firmware and distributing outgoing packets to the correct
 * hardware interface.
 */
class CommModule {
private:
public:
    /// Type aliases
    using RxCallbackSigT = void(RTP::Packet);
    using TxCallbackSigT = int32_t(const RTP::Packet*);
    using RxCallbackT = std::function<RxCallbackSigT>;
    using TxCallbackT = std::function<TxCallbackSigT>;
    using PortT = CommPort<RxCallbackSigT, TxCallbackSigT>;

    /// Class constants
    static constexpr size_t TX_QUEUE_SIZE = 3;
    static constexpr size_t RX_QUEUE_SIZE = 3;

    /// global singleton instance of CommModule
    static std::shared_ptr<CommModule> Instance;

    /// The constructor initializes and starts threads and mail queues
    CommModule(std::shared_ptr<FlashingTimeoutLED> rxTimeoutLED,
               std::shared_ptr<FlashingTimeoutLED> txTimeoutLED);

    /// The destructor frees up allocated memory and stops threads
    ~CommModule(){};

    /// initializes and starts rx/tx threads and mail queues
    void init();

    /// Assign an RX callback function to a port
    void setRxHandler(RxCallbackT callback, uint8_t portNbr);

    /// Assign a TX callback function to a port
    void setTxHandler(TxCallbackT callback, uint8_t portNbr);

    /// Assign an RX callback method to a port
    template <typename B>
    void setRxHandler(B* obj, void (B::*mptr)(RTP::Packet), uint8_t portNbr) {
        setRxHandler(std::bind(mptr, obj, std::placeholders::_1), portNbr);
    }

    /// Assign an TX callback method to a port
    template <typename B>
    void setTxHandler(B* obj, int32_t (B::*mptr)(const RTP::Packet*),
                      uint8_t portNbr) {
        setTxHandler(std::bind(mptr, obj, std::placeholders::_1), portNbr);
    }

    /// Send an RTP::Packet over previsouly initialized hardware
    void send(RTP::Packet pkt);

    /// Called by CommLink instances whenever a packet is received via radio
    void receive(RTP::Packet pkt);

    /// Close a port that was previouly assigned callback functions/methods
    void close(unsigned int portNbr) { m_ports.erase(portNbr); }

    /// Check if everything is ready for sending/receiving packets
    bool isReady() const { return m_isReady; };

    int numOpenSockets() const;

    unsigned int numRxPackets() const;
    unsigned int numTxPackets() const;

    void resetCount(unsigned int portNbr);

#ifndef NDEBUG
    void printInfo() const;
#endif

protected:
    // Memory Queue IDs
    osMailQId m_txQueue;
    osMailQId m_rxQueue;

private:
    static constexpr size_t STACK_SIZE = DEFAULT_STACK_SIZE / 2;

    std::map<uint8_t, PortT> m_ports;

    Thread m_rxThread;
    Thread m_txThread;

    MailHelper<RTP::Packet, TX_QUEUE_SIZE> m_txQueueHelper;
    MailHelper<RTP::Packet, RX_QUEUE_SIZE> m_rxQueueHelper;

    std::shared_ptr<FlashingTimeoutLED> m_rxTimeoutLED;
    std::shared_ptr<FlashingTimeoutLED> m_txTimeoutLED;

    bool m_isReady = false;

    void ready();

    void txThread();
    void rxThread();

    /// The threadHelper methods accept a CommModule pointer as a parameter
    /// and call the corresponding instance methods on the module.
    inline void rxThreadHelper(const void* moduleInst) {
        auto module = reinterpret_cast<CommModule*>(
            const_cast<void*>(moduleInst));  // dangerous
        module->rxThread();
    }
    inline void txThreadHelper(CommModule* const moduleInst) {
        auto module = reinterpret_cast<CommModule*>(
            const_cast<void*>(moduleInst));  // dangerous
        module->txThread();
    }
};
