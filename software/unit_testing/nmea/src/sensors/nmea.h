/**
 * @file nmea.h
 * @brief Robust, zero-copy NMEA 0183 parser for embedded systems.
 * Supports GPGGA, GPRMC, and their GN/GL/BD/GA variants.
 */
#ifndef NMEA_PARSER_HPP
#define NMEA_PARSER_HPP

#include <cstdint>
#include <array>
#include <optional>
#include <string_view>

namespace firmware::sensors {

struct GpsData {
    double latitude = 0.0;   ///< Decimal Degrees (Positive North, Negative South)
    double longitude = 0.0;  ///< Decimal Degrees (Positive East, Negative West)
    float altitude = 0.0f;   ///< Meters above Mean Sea Level
    float speed = 0.0f;      ///< Ground Speed in m/s
    float course = 0.0f;     ///< Course over ground in degrees
    uint8_t satellites = 0;  ///< Number of satellites used in solution
    uint8_t fixType = 0;     ///< 0=No Fix, 1=GPS Fix, 2=Diff Fix
    bool valid = false;      ///< True if the last packet indicated valid data
    uint32_t lastUpdateTick = 0; ///< System tick of last successful parse
};

class NmeaParser {
public:
    NmeaParser() = default;

    /**
     * @brief Feeds a single byte into the parser state machine.
     * @return true if a complete packet was successfully parsed.
     */
    bool processByte(uint8_t byte);

    /**
     * @brief Returns the most recent valid GPS data.
     */
    GpsData getLatestData() const;

private:
    static constexpr size_t MAX_PACKET_SIZE = 85;

    std::array<char, MAX_PACKET_SIZE> m_buffer;
    size_t m_bufferIndex = 0;
    bool m_inPacket = false;
    GpsData m_currentData;

    void parsePacket(std::string_view packet);
    bool validateChecksum(std::string_view packet) const;
    void parseGga(std::string_view body);
    void parseRmc(std::string_view body);

    // Modern C++23 helpers
    double nmeaToDecimal(std::string_view val, std::string_view quadrant);
    std::string_view getToken(std::string_view str, size_t index) const;
    float parseFloat(std::string_view str) const;
    double parseDouble(std::string_view str) const;
    int parseInt(std::string_view str) const;
};

} // namespace firmware::sensors

#endif // NMEA_PARSER_HPP
