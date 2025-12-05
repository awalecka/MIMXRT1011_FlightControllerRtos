#ifndef IBUS_PROTOCOL_HPP
#define IBUS_PROTOCOL_HPP

#include <cstdint>
#include <array>
#include <stddef.h>

/**
 * @file ibus_protocol.hpp
 * @brief Defines constants and structures for the FlySky IBUS RC protocol.
 */

namespace firmware::protocols::ibus {

/**
 * @brief IBUS protocol constants.
 */
constexpr uint8_t PACKET_LENGTH = 32U;
constexpr uint8_t COMMAND_SERVO = 0x40U;
constexpr size_t CHANNEL_COUNT = 14U;
constexpr uint32_t BAUD_RATE = 115200U;

/**
 * @brief Raw packed structure representing the IBUS packet over the wire.
 * * @note This uses C-style snake_case for member variables as per style guide for structs.
 * We use __attribute__((packed)) to ensure strict alignment to the byte stream.
 */
struct IbusPacketRaw {
    uint8_t length;                 ///< Protocol length (usually 0x20)
    uint8_t command;                ///< Command type (usually 0x40)
    uint16_t channels[CHANNEL_COUNT]; ///< Channel data (Little Endian)
    uint16_t checksum;              ///< Checksum (0xFFFF - sum)
} __attribute__((packed));

static_assert(sizeof(IbusPacketRaw) == PACKET_LENGTH, "IbusPacketRaw size mismatch");

} // namespace firmware::protocols::ibus

#endif // IBUS_PROTOCOL_HPP
