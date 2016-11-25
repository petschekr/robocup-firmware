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
    // Type aliases
    using RxCallbackSigT = void(RTP::Packet);
    using TxCallbackSigT = int32_t(const RTP::Packet*);
    using RxCallbackT = std::function<RxCallbackSigT>;
    using TxCallbackT = std::function<TxCallbackSigT>;
    using PortT = CommPort<RxCallbackSigT, TxCallbackSigT>;

    // Class constants
    // Be careful of the queue sizes. The errors that result from
    // over allocation are very tricky to catch.
    static constexpr size_t TX_QUEUE_SIZE = 3;
    static constexpr size_t RX_QUEUE_SIZE = 3;

    /// The constructor initializes and starts threads and mail queues
    CommModule(std::shared_ptr<FlashingTimeoutLED> rxTimeoutLED,
               std::shared_ptr<FlashingTimeoutLED> txTimeoutLED);

    /// The destructor frees up allocated memory and stops threads
    ~CommModule(){};

    /// global singleton instance of CommModule
    static std::shared_ptr<CommModule> Instance;

    /// initializes and starts rx/tx threads and mail queues
    void init();

    // Set a TX callback function on an object
    template <typename B>
    void setTxHandler(B* obj, int32_t (B::*mptr)(const RTP::Packet*),
                      uint8_t portNbr) {
        setTxHandler(std::bind(mptr, obj, std::placeholders::_1), portNbr);
    }

    // Set an RX callback function on an object
    template <typename B>
    void setRxHandler(B* obj, void (B::*mptr)(RTP::Packet), uint8_t portNbr) {
        setRxHandler(std::bind(mptr, obj, std::placeholders::_1), portNbr);
    }

    // Set a normal RX callback function without an object
    void setRxHandler(RxCallbackT callback, uint8_t portNbr);
    void setTxHandler(TxCallbackT callback, uint8_t portNbr);

    // Send a RTP::Packet. The details of exactly how the packet will be sent
    // are determined from the RTP::Packet's port
    void send(RTP::Packet packet);

    /// Called by CommLink instances whenever a packet is received via radio
    void receive(RTP::Packet pkt);

    unsigned int numRxPackets() const;
    unsigned int numTxPackets() const;
    void resetCount(unsigned int portNbr);

    void printInfo() const;

    void close(unsigned int portNbr) { m_ports.erase(portNbr); }
    bool isReady() const { return m_isReady; };
    int numOpenSockets() const;

protected:
    // Memory Queue IDs
    osMailQId m_txQueue;
    osMailQId m_rxQueue;

private:
    std::map<uint8_t, PortT> m_ports;

    Thread m_rxThread;
    Thread m_txThread;

    // Mail helper objects
    MailHelper<RTP::Packet, TX_QUEUE_SIZE> m_txQueueHelper;
    MailHelper<RTP::Packet, RX_QUEUE_SIZE> m_rxQueueHelper;

    std::shared_ptr<FlashingTimeoutLED> m_rxTimeoutLED, m_txTimeoutLED;

    bool m_isReady = false;

    // The working threads for handling rx and tx data queues
    void txThread();
    void rxThread();

    /// The threadHelper methods accept a CommModule pointer as a parameter
    /// and call the corresponding instance methods on the module.
    static void rxThreadHelper(const void* moduleInst);
    static void txThreadHelper(const void* moduleInst);

    void ready();
};
