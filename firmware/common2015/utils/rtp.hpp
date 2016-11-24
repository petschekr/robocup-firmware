#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rtp {

/// Max packet size.  This is limited by the CC1201 buffer size.
static constexpr size_t MAX_DATA_SZ = 120;

constexpr uint8_t BROADCAST_ADDRESS = 0x00;  // configured by the PKT_CFG1 register
constexpr uint8_t BASE_STATION_ADDRESS = 0xFF - 1;
constexpr uint8_t ROBOT_ADDRESS = 0x01;  // All robots have the same address
constexpr uint8_t LOOPBACK_ADDRESS = 2;

// The value 0 is a valid robot id, so we have to choose something else to
// represent "null"
const uint8_t INVALID_ROBOT_UID = 0xFF;

template <typename PACKET_TYPE>
void SerializeToVector(const PACKET_TYPE& pkt, std::vector<uint8_t>* buf) {
    const auto bytes = reinterpret_cast<const uint8_t*>(&pkt);
    for (auto i = 0; i < sizeof(PACKET_TYPE); ++i) {
        buf->push_back(bytes[i]);
    }
}

template <typename PACKET_TYPE>
bool DeserializeFromBuffer(PACKET_TYPE* pkt, uint8_t* buf, size_t bufSize) {
    if (bufSize < sizeof(PACKET_TYPE)) return false;
    memcpy(pkt, buf, bufSize);
    return true;
}

/**
 * @brief Port enumerations for different communication protocols.
 */
enum Port { SINK = 0, LINK = 1, CONTROL = 2, LEGACY = 3, PING = 4 };

struct header_data {
    enum Type { Control, Tuning, FirmwareUpdate, Misc };

    header_data(Port p = SINK) : address(0), port(p), type(Control){};

    uint8_t address;
    Port port : 4;
    Type type : 4;
} __attribute__((packed));

// binary-packed version of Control.proto
struct ControlMessage {
    uint8_t uid;  // robot id
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
};

/**
 * @brief Real-Time packet definition
 */
class packet {
public:
    rtp::header_data header;
    std::vector<uint8_t> payload;

    static constexpr size_t headerSize = sizeof(header);

    packet(){};
    packet(const std::string& s, Port p = SINK) : header(p) {
        payload.reserve(s.size() + 1);
        for (char c : s) payload.push_back(c);
        payload.push_back('\0');
    }

    template <class T>
    packet(const std::vector<T>& v, Port p = SINK) : header(p) { recv(v); }

    size_t size() const { return headerSize + payload.size(); }

    /// deserialize a packet from a buffer
    template <class T>
    void recv(const std::vector<T>& v) {
        // check that the buffer is big enough
        if (v.size() < headerSize) return;

        // deserialize header
        header = *(reinterpret_cast<const header_data*>(v.data()));

        // copy over the payload
        payload.clear();
        payload.insert(payload.begin(), v.begin() + headerSize, v.end());
    }

    void pack(std::vector<uint8_t>* buffer) const {
        buffer->reserve(size());
        SerializeToVector(header, buffer);
        buffer->insert(buffer->end(), payload.begin(), payload.end());
    }
};

// Packet sizes
constexpr size_t Forward_Size = packet::headerSize + 6 * sizeof(ControlMessage);
constexpr size_t Reverse_Size = packet::headerSize + sizeof(RobotStatusMessage);

}  // namespace rtp
