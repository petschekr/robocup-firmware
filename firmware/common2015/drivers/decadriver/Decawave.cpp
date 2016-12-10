#include "Decawave.hpp"

#include "Assert.hpp"
#include "Logger.hpp"

#include <memory>

namespace {
dwt_config_t config = {
    4,                 /* Channel number. */
    DWT_PRF_64M,       /* Pulse repetition frequency. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    17,                /* TX preamble code. Used in TX only. */
    17,                /* RX preamble code. Used in RX only. */
    1,                 /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    (128 + 1 + 64 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC
                          size). Used in RX only. */
};

dwt_txconfig_t txconfig = {
    0x95,       /* PG delay. */
    0x9A9A9A9A, /* TX power. */
};

constexpr auto TX_TO_RX_DELAY_UUS = 60;
constexpr auto RX_RESP_TO_UUS = 5000;
}

Decawave::Decawave(SpiPtrT sharedSPI, PinName nCs, PinName intPin)
    : CommLink(sharedSPI, nCs, intPin), dw1000_api() {
    setSPIFrequency(2'000'000);

    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        LOG(SEVERE, "Decawave not initialized");
        return;
    }

    reset();
    selfTest();

    if (m_isInit) {
        dwt_configure(&config);
        dwt_configuretxrf(&txconfig);

        setSPIFrequency(8'000'000);  // won't work after 8MHz

        dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);
        dwt_setcallbacks(
            nullptr, static_cast<dwt_cb_t>(&Decawave::getDataSuccess), nullptr,
            static_cast<dwt_cb_t>(&Decawave::getDataFail));
        dwt_setinterrupt(DWT_INT_RFCG, 1);

        dwt_setautorxreenable(1);

        setLED(true);
        dwt_forcetrxoff();  // TODO: Better way than force off then reset?
        dwt_rxreset();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        LOG(INFO, "Decawave ready!");
        CommLink::ready();
    }
}

// Virtual functions from CommLink
int32_t Decawave::sendPacket(const RTP::Packet* pkt) {
    // Return failutre if not initialized
    if (!m_isInit) return COMM_FAILURE;

    dwt_rxreset();
    dwt_forcetrxoff();

    const auto bufferSize = 9 + pkt->size() + 2;
    BufferT txBuffer(bufferSize);

    // MAC layer header for Decawave
    txBuffer.insert(txBuffer.end(),
                    {0x41, 0x88, 0x00, 0xCA, 0xDE, pkt->header.address, 0x00,
                     static_cast<uint8_t>(m_address), 0x00});

    ASSERT(txBuffer.size() == 9);

    const auto headerFirstPtr = reinterpret_cast<const uint8_t*>(&pkt->header);
    const auto headerLastPtr = headerFirstPtr + RTP::HeaderSize - 1;

    // insert the RTP header
    txBuffer.insert(txBuffer.end(), headerFirstPtr, headerLastPtr);
    // insert the RTP payload
    txBuffer.insert(txBuffer.end(), pkt->payload.begin(), pkt->payload.end());
    // insert padding for CRC
    txBuffer.insert(txBuffer.end(), {0x00, 0x00});

    // send to the Decawave's API
    dwt_writetxdata(txBuffer.size(), txBuffer.data(), 0);
    dwt_writetxfctrl(txBuffer.size(), 0, 0);

    const auto sentStatus =
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    const auto hadError = sentStatus != DWT_SUCCESS;

    return hadError ? COMM_DEV_BUF_ERR : COMM_SUCCESS;
}

CommLink::BufferT Decawave::getData() {
    BufferT buf{};

    // Return empty data if not initialized
    if (!m_isInit) return std::move(buf);

    // set the m_rxBuffer's pointer to our vector before calling the isr
    // function
    m_rxBufferPtr = &buf;

    // manually invoke the isr routine & set back into RX mode
    dwt_isr();
    // our buffer is now filled with the received bytes if everything went ok
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // return empty buffer if the isr routine failed to fill it with anything
    if (buf.empty()) return std::move(buf);

    // remove the last 2 elements
    buf.erase(buf.end() - 3, buf.end() - 1);

    // move the buffer to the caller
    return std::move(buf);
}

int32_t Decawave::selfTest() {
    m_chipVersion = dwt_readdevid();

    if (m_chipVersion != DWT_DEVICE_ID) {
        LOG(SEVERE,
            "Decawave part number error:\r\n"
            "    Found:\t0x%02X (expected 0x%02X)",
            m_chipVersion, DWT_DEVICE_ID);

        return -1;
    } else {
        m_isInit = true;
        return 0;
    }
}

void Decawave::setAddress(int addr) {
    CommLink::setAddress(addr);

    dwt_setpanid(0xDECA);
    dwt_setaddress16(static_cast<uint16_t>(addr));
    dwt_enableframefilter(DWT_FF_DATA_EN);
}

// Virtual functions from dw1000_api
int Decawave::writetospi(uint16 headerLength, const uint8* headerBuffer,
                         uint32 bodylength, const uint8* bodyBuffer) {
    chipSelect();
    for (size_t i = 0; i < headerLength; ++i) _spi->write(headerBuffer[i]);
    for (size_t i = 0; i < bodylength; ++i) _spi->write(bodyBuffer[i]);
    chipDeselect();
    return 0;
}

int Decawave::readfromspi(uint16 headerLength, const uint8* headerBuffer,
                          uint32 readlength, uint8* readBuffer) {
    chipSelect();
    for (size_t i = 0; i < headerLength; ++i) _spi->write(headerBuffer[i]);
    for (size_t i = 0; i < readlength; ++i) readBuffer[i] = _spi->write(0);
    chipDeselect();
    return 0;
}

// Callback functions for decawave interrupt
void Decawave::getDataSuccess(const dwt_cb_data_t* cb_data) {
    const auto offset = 9;
    const auto dataLength = cb_data->datalength - offset;
    // Read recived data to m_rxBufferPtr array
    m_rxBufferPtr->reserve(dataLength);
    dwt_readrxdata(m_rxBufferPtr->data(), dataLength, offset);
}

void Decawave::getDataFail(const dwt_cb_data_t* cb_data) {}
