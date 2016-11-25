#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace RTP {

constexpr uint8_t BROADCAST_ADDRESS =
    0x00;  // configured by the PKT_CFG1 register
constexpr uint8_t BASE_STATION_ADDRESS = 0xFF - 1;
constexpr uint8_t ROBOT_ADDRESS = 0x01;  // All robots have the same address
constexpr uint8_t LOOPBACK_ADDRESS = 2;

// The value 0 is a valid robot id, so we have to choose something else to
// represent "null"
constexpr uint8_t INVALID_ROBOT_UID = 0xFF;

template <typename PACKET_TYPE>
void serializeToVector(const PACKET_TYPE& pkt, std::vector<uint8_t>* buf) {
    const auto data = reinterpret_cast<const uint8_t*>(&pkt);
    buf->reserve(sizeof(PACKET_TYPE));
    for (size_t i = 0; i < sizeof(PACKET_TYPE); ++i) {
        buf->push_back(data[i]);
    }
}

// a hackish way of enforcing 'enum class' scopes without
// the bitfield restrictions
namespace PortTypeNamespace {
enum PortTypeEnum { SINK, LINK, CONTROL, LEGACY, PING };
}
using PortType = PortTypeNamespace::PortTypeEnum;

namespace MessageTypeNamespace {
enum MessageTypeEnum { CONTROL, TUNING, UPGRADE, MISC };
}
using MessageType = MessageTypeNamespace::MessageTypeEnum;

struct Header {
    Header(PortType p = PortType::SINK)
        : address(0), port(p), type(MessageType::CONTROL) {}

    uint8_t address;
    PortType port : 4;
    MessageType type : 4;
} __attribute__((packed));

// binary-packed version of Control.proto
struct ControlMessage {
    uint8_t uid;  // robot id

    /** body{X,Y,W} are multiplied by this value before being sent over the
     * radio and must be then divided by this value on the receiving side. This
     * is to avoid loss of precision when sending float velocity values across
     * the air as ints.
     */
    static const uint16_t VELOCITY_SCALE_FACTOR = 1000;

    int16_t bodyX;
    int16_t bodyY;
    int16_t bodyW;
    int8_t dribbler;
    uint8_t kickStrength;
    unsigned shootMode : 1;    // 0 = kick, 1 = chip
    unsigned triggerMode : 2;  // 0 = off, 1 = immediate, 2 = on break beam
    unsigned song : 2;         // 0 = stop, 1 = continue, 2 = GT fight song
} __attribute__((packed));

struct RobotStatusMessage {
    uint8_t uid;  // robot id

    /** @battVoltage is a direct reading from the mbed's ADC and is sent over
     * the air as-is.  Soccer must convert this reading into an actual voltage
     * value by multiplying it by the scale factor. The theoretical scale factor
     * is 0.100546875, but this has been adjusted after testing to the value
     * below.
     */
    static constexpr float BATTERY_READING_SCALE_FACTOR = 0.09884;

    uint8_t battVoltage;
    uint8_t ballSenseStatus : 2;
    uint8_t motorErrors : 5;  // 1 bit for each motor - 1 = error, 0 = good
    uint8_t fpgaStatus : 2;   // 0 = good, 1 = not initialized, 2 = error
};

/**
 * @brief Real-Time packet definition
 */
class Packet {
public:
    RTP::Header header;
    std::vector<uint8_t> payload;

    Packet(){};

    template <typename T, typename = std::enable_if_t<
                              std::is_convertible<T, uint8_t>::value>>
    Packet(const std::vector<T>& v, PortType p = PortType::SINK) : header(p) {
        recv(v);
    }

    Packet(const std::string& s, PortType p = PortType::SINK) : header(p) {
        payload.assign(s.begin(), s.end());
        payload.push_back('\0');
    }

    /// deserialize a packet from a buffer
    template <typename T, typename = std::enable_if_t<
                              std::is_convertible<T, uint8_t>::value>>
    void recv(const std::vector<T>& buf) {
        // check that the buffer is big enough
        if (buf.size() >= sizeof(Header)) {
            // deserialize header
            header = *(reinterpret_cast<const Header*>(buf.data()));
            // set the payload bytes
            payload.assign(buf.begin() + sizeof(Header), buf.end());
        }
    }

    template <typename T, typename = std::enable_if_t<
                              std::is_convertible<T, uint8_t>::value>>
    void pack(std::vector<T>* buf) const {
        buf->reserve(size());
        serializeToVector(header, buf);
        buf->insert(buf->end(), payload.begin(), payload.end());
    }

    size_t size() const { return sizeof(Header) + payload.size(); }
};

// Packet sizes
constexpr auto ForwardSize = sizeof(Header) + 6 * sizeof(ControlMessage);
constexpr auto ReverseSize = sizeof(Header) + sizeof(RobotStatusMessage);

}  // namespace RTP
