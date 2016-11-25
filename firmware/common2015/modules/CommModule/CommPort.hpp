#pragma once

#include <mbed.h>
#include <rtos.h>
#include "logger.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <stdexcept>

template <typename RX_CALLBACK, typename TX_CALLBACK>
class CommPort {
public:
    using RxCallbackT = std::function<RX_CALLBACK>;
    using TxCallbackT = std::function<TX_CALLBACK>;
    using RxCallbackRefT = RxCallbackT&;
    using TxCallbackRefT = TxCallbackT&;
    using ConstRxCallbackRefT = const RxCallbackT&;
    using ConstTxCallbackRefT = const TxCallbackT&;

    /// Counters for the number of packets sent/received via this port
    unsigned int m_rxCount = 0;
    unsigned int m_txCount = 0;

    CommPort(RxCallbackT rxC = nullptr, TxCallbackT txC = nullptr)
        : m_rxCallback(rxC), m_txCallback(txC) {}

    // Set functions for each RX/TX callback.
    void setRxCallback(ConstRxCallbackRefT func) { m_rxCallback = func; }
    void setTxCallback(ConstTxCallbackRefT func) { m_txCallback = func; }

    /// Methods that return a reference to the TX/RX callback function pointers
    RxCallbackRefT rxCallback() { return m_rxCallback; }
    ConstRxCallbackRefT rxCallback() const { return m_rxCallback; }

    TxCallbackRefT txCallback() { return m_txCallback; }
    ConstTxCallbackRefT txCallback() const { return m_txCallback; }

    // Returns the current packet counts to zero
    void resetPacketCount() {
        m_rxCount = 0;
        m_txCount = 0;
    }

private:
    // the class members that hold the function pointers
    RxCallbackT m_rxCallback = nullptr;
    TxCallbackT m_txCallback = nullptr;
};
