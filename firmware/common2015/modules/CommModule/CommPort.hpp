#pragma once

#include <functional>

template <typename RX_CALLBACK, typename TX_CALLBACK>
class CommPort {
public:
    using RxCallbackT = std::function<RX_CALLBACK>;
    using TxCallbackT = std::function<TX_CALLBACK>;
    using ConstRxCallbackRefT = const RxCallbackT&;
    using ConstTxCallbackRefT = const TxCallbackT&;

    CommPort(RxCallbackT rxC = nullptr, TxCallbackT txC = nullptr)
        : m_rxCallback(rxC), m_txCallback(txC) {}

    /// Bind an RX callback function
    void setRxCallback(ConstRxCallbackRefT func) { m_rxCallback = func; }

    /// Bind a TX callback function
    void setTxCallback(ConstTxCallbackRefT func) { m_txCallback = func; }

    /// Returns a reference to the RX callback function
    ConstRxCallbackRefT getRxCallback() {
        ++m_rxCount;
        return m_rxCallback;
    }

    /// Returns a reference to the TX callback function
    ConstTxCallbackRefT getTxCallback() {
        ++m_txCount;
        return m_txCallback;
    }

    /// Returns true if an RX callback is available
    bool hasRxCallback() const { return m_rxCallback != nullptr; }

    /// Returns true if a TX callback is available
    bool hasTxCallback() const { return m_txCallback != nullptr; }

    /// Returns the number of times the RX callback has been asked for
    unsigned int getRxCount() const { return m_rxCount; }

    /// Returns the number of times the TX callback has been asked for
    unsigned int getTxCount() const { return m_txCount; }

    /// Resets the current packet counts to zero
    void resetCounts() {
        m_rxCount = 0;
        m_txCount = 0;
    }

private:
    /// Counters for the number of times callback is asked for
    unsigned int m_rxCount = 0;
    unsigned int m_txCount = 0;

    /// The function pointer holders
    RxCallbackT m_rxCallback = nullptr;
    TxCallbackT m_txCallback = nullptr;
};
