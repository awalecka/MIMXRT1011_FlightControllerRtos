#ifndef IBUS_HANDLER_HPP
#define IBUS_HANDLER_HPP

#include "ibus_protocol.hpp"
#include <span>
#include <array>

namespace firmware::drivers {

/**
 * @class IbusHandler
 * @brief Handles parsing, validation, and data extraction for IBUS RC packets.
 * * Design for NXP RT1011:
 * - No heap allocation.
 * - Uses C++23 std::span for zero-copy buffer processing.
 * - Designed to be called from a FreeRTOS task or ISR context (keep processing short).
 */
class IbusHandler {
public:
    /**
     * @brief Default constructor. Initializes channels to center value (1500).
     */
    IbusHandler() noexcept;

    /**
     * @brief Processes a raw byte buffer received via DMA.
     * * @param buffer A view of the received bytes (must be 32 bytes).
     * @return true if packet was valid and channels updated, false otherwise.
     */
    bool processBuffer(std::span<const uint8_t> buffer) noexcept;

    /**
     * @brief Retrieves the latest value for a specific channel.
     * * @param channelIndex The index of the channel (0-13).
     * @return uint16_t The channel value (typically 1000-2000), or 0 if index invalid.
     */
    uint16_t getChannel(size_t channelIndex) const noexcept;

    /**
     * @brief Returns a read-only reference to the entire channel array.
     * * @return const std::array<uint16_t, protocols::ibus::CHANNEL_COUNT>&
     */
    const std::array<uint16_t, protocols::ibus::CHANNEL_COUNT>& getAllChannels() const noexcept;

private:
    /**
     * @brief Calculates the IBUS checksum.
     * * @param buffer The buffer to calculate over (excluding the checksum bytes themselves).
     * @return uint16_t The calculated checksum.
     */
    uint16_t calculateChecksum(std::span<const uint8_t> buffer) const noexcept;

    /**
     * @brief Internal storage for decoded channel data.
     */
    std::array<uint16_t, protocols::ibus::CHANNEL_COUNT> m_channelData;

    /**
     * @brief Counter for invalid packets (useful for debugging telemetry).
     */
    uint32_t m_errorCount;
};

} // namespace firmware::drivers

#endif // IBUS_HANDLER_HPP
