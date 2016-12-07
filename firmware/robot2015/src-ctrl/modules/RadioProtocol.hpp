#pragma once

#include <rtos.h>
#include "CC1201.hpp"
#include "CommModule.hpp"
#include "Decawave.hpp"
#include "RtosTimerHelper.hpp"

class RadioProtocol {
public:
    enum class State {
        STOPPED,
        DISCONNECTED,
        CONNECTED,
    };

    /// After this number of milliseconds without receiving a packet from the
    /// base station, we are considered "disconnected"
    static const uint32_t TIMEOUT_INTERVAL = 2000;

    RadioProtocol(std::shared_ptr<CommModule> commModule,
                  uint8_t uid = RTP::INVALID_ROBOT_UID)
        : _commModule(commModule),
          _uid(uid),
          _state(State::STOPPED),
          _replyTimer(this, &RadioProtocol::reply, osTimerOnce),
          _timeoutTimer(this, &RadioProtocol::_timeout, osTimerOnce) {
        ASSERT(commModule != nullptr);
        ASSERT(globalRadio != nullptr);
        globalRadio->setAddress(RTP::ROBOT_ADDRESS);
    }

    ~RadioProtocol() { stop(); }

    /// Set robot unique id.  Also update address.
    void setUID(uint8_t uid) { _uid = uid; }

    /**
     * Callback that is called whenever a packet is received.  Set this in
     * order to handle parsing the packet and creating a response.  This
     * callback
     * should return a formatted reply buffer, which will be sent in the
     * appropriate reply slot.
     *
     * @param msg A pointer to the start of the message addressed to this robot
     * @return formatted reply buffer
     */
    std::function<std::vector<uint8_t>(const RTP::ControlMessage* msg,
                                       const bool addresed)>
        rxCallback;

    void start() {
        _state = State::DISCONNECTED;

        _commModule->setRxHandler(this, &RadioProtocol::rxHandler,
                                  RTP::PortType::CONTROL);
        _commModule->setTxHandler(globalRadio.get(), &CommLink::sendPacket,
                                  RTP::PortType::CONTROL);

        LOG(INFO, "Radio protocol listening on port %d",
            RTP::PortType::CONTROL);
    }

    void stop() {
        _commModule->close(RTP::PortType::CONTROL);

        _replyTimer.stop();
        _state = State::STOPPED;

        LOG(INFO, "Radio protocol stopped");
    }

    State state() const { return _state; }

    void rxHandler(RTP::Packet pkt) {
        // TODO: check packet size before parsing
        bool addressed = false;
        const RTP::ControlMessage* msg;
        size_t slot;
        // printf("UUIDs: ");
        for (slot = 0; slot < 6; slot++) {
            const auto offset = slot * sizeof(RTP::ControlMessage);
            msg = reinterpret_cast<const RTP::ControlMessage*>(
                pkt.payload.data() + offset);

            // printf("%d:%d ", slot, msg->uid);
            if (msg->uid == _uid) {
                addressed = true;
                break;
            }
        }
        // printf("\r\n");

        /// time, in ms, for each reply slot
        // TODO(justin): double-check this
        const uint32_t SLOT_DELAY = 2;

        _state = State::CONNECTED;

        // reset timeout whenever we receive a packet
        _timeoutTimer.stop();
        _timeoutTimer.start(TIMEOUT_INTERVAL);

        _replyTimer.start(1 + SLOT_DELAY * slot);

        if (rxCallback) {
            _reply = std::move(rxCallback(msg, addressed));
        } else {
            LOG(WARN, "no callback set");
        }
    }

private:
    void reply() {
        RTP::Packet pkt;
        pkt.header.port = RTP::PortType::CONTROL;
        pkt.header.type = RTP::MessageType::CONTROL;
        pkt.header.address = RTP::BASE_STATION_ADDRESS;

        pkt.payload = std::move(_reply);

        _commModule->send(std::move(pkt));
    }

    void _timeout() { _state = State::DISCONNECTED; }

    std::shared_ptr<CommModule> _commModule;

    uint32_t _lastReceiveTime = 0;

    uint8_t _uid;
    State _state;

    std::vector<uint8_t> _reply;

    RtosTimerHelper _replyTimer;
    RtosTimerHelper _timeoutTimer;
};
