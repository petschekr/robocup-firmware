#pragma once

#include "CommLink.hpp"
#include "mbed.h"
#include "rtos.h"

#include "deca_device_api.hpp"
#include "deca_regs.h"

class Decawave : public CommLink, public dw1000_api {
public:
    Decawave(spiPtr_t sharedSPI, PinName nCs, PinName intPin);

    int32_t sendPacket(const RTP::Packet* pkt) override;
    BufferT getData() override;
    void reset() override { dwt_softreset(); }
    int32_t selfTest() override;
    bool isConnected() const override { return _isInit; }

    int writetospi(uint16 headerLength, const uint8* headerBuffer,
                   uint32 bodylength, const uint8* bodyBuffer);
    int readfromspi(uint16 headerLength, const uint8* headerBuffer,
                    uint32 readlength, uint8* readBuffer);
    decaIrqStatus_t decamutexon() { return 0; }

#if 0
    void decamutexoff(decaIrqStatus_t s);
    void deca_sleep(unsigned int time_ms);
#endif

    void setAddress(uint16_t addr);
    void setLED(bool ledOn) { dwt_setleds(ledOn); };

private:
    BufferPtrT rx_buffer = nullptr;
    BufferPtrT tx_buffer = nullptr;
    uint32_t _chip_version;
    uint8_t _addr = RTP::INVALID_ROBOT_UID;
    bool _isInit = false;

    void getData_success(const dwt_cb_data_t* cb_data);
    void getData_fail(const dwt_cb_data_t* cb_data);
};

extern Decawave* global_radio;
