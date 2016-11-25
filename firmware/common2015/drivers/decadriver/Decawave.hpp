#pragma once

#include "CommLink.hpp"
#include "deca_device_api.hpp"

class Decawave : public CommLink, public dw1000_api {
public:
    Decawave(SpiPtrT sharedSPI, PinName nCs, PinName intPin);

    int32_t sendPacket(const RTP::Packet* pkt) override;

    BufferT getData() override;

    void reset() override { dwt_softreset(); }

    int32_t selfTest() override;

    bool isConnected() const override { return m_isInit; }

    int writetospi(uint16 headerLength, const uint8* headerBuffer,
                   uint32 bodylength, const uint8* bodyBuffer) override;
    int readfromspi(uint16 headerLength, const uint8* headerBuffer,
                    uint32 readlength, uint8* readBuffer) override;
    decaIrqStatus_t decamutexon() override { return 0; }

#if 0
    void decamutexoff(decaIrqStatus_t s);
    void deca_sleep(unsigned int time_ms);
#endif

    void setAddress(uint16_t addr);
    void setLED(bool ledOn) { dwt_setleds(ledOn); };

private:
    BufferPtrT m_rxBufferPtr = nullptr;
    BufferPtrT m_txBufferPtr = nullptr;
    uint32_t m_chipVersion;
    uint8_t m_addr = RTP::INVALID_ROBOT_UID;
    bool m_isInit = false;

    void getDataSuccess(const dwt_cb_data_t* cb_data);
    void getDataFail(const dwt_cb_data_t* cb_data);
};
