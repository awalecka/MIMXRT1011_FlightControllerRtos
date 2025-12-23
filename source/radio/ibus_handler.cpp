#include "ibus_handler.hpp"
#include <cstring>
#include <algorithm>

namespace firmware::drivers {

using namespace firmware::protocols::ibus;

IbusHandler::IbusHandler() noexcept : m_errorCount(0) {
    // Initialize channels to a safe "center" value (1500us)
    // std::fill is safe and standard
    std::fill(m_channelData.begin(), m_channelData.end(), 1500);
}

bool IbusHandler::processBuffer(std::span<const uint8_t> buffer) noexcept {
    // Basic Size Validation
    if (buffer.size() < PACKET_LENGTH) {
        m_errorCount++;
        return false;
    }

    // Cast to raw struct pointer for easy field access.
    // In C++23/embedded, we must ensure alignment.
    // The Cortex-M7 (RT1011) handles unaligned access, but reinterpret_cast
    // on a byte buffer to a packed struct is the standard embedded pattern here.
    const auto* packet = reinterpret_cast<const IbusPacketRaw*>(buffer.data());

    // Header Validation
    if (packet->length != PACKET_LENGTH || packet->command != COMMAND_SERVO) {
        m_errorCount++;
        return false;
    }

    // Checksum Validation
    // The checksum is over the first 30 bytes (Length + Command + 14 channels)
    // The last 2 bytes are the checksum itself.
    uint16_t calculatedSum = calculateChecksum(buffer.first(PACKET_LENGTH - 2));

    if (calculatedSum != packet->checksum) {
        m_errorCount++;
        return false;
    }

    // Update Channel Data
    // We iterate manually or use std::copy. Since the packed struct is Little Endian
    // and the RT1011 is Little Endian, we can copy directly.
    for (size_t i = 0; i < CHANNEL_COUNT; ++i) {
        m_channelData[i] = packet->channels[i];
    }

    return true;
}

uint16_t IbusHandler::getChannel(size_t channelIndex) const noexcept {
    if (channelIndex >= CHANNEL_COUNT) {
        return 0;
    }
    return m_channelData[channelIndex];
}

const std::array<uint16_t, CHANNEL_COUNT>& IbusHandler::getAllChannels() const noexcept {
    return m_channelData;
}

uint16_t IbusHandler::calculateChecksum(std::span<const uint8_t> buffer) const noexcept {
    uint16_t sum = 0xFFFF;

    for (uint8_t byte : buffer) {
        sum -= byte;
    }

    return sum;
}

} // namespace firmware::drivers
