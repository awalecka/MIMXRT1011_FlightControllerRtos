/**
 * @file nmea.cpp
 * @brief Implementation of the NMEA parser.
 */
#include "nmea.h"
#include <charconv>
#include <cmath>
#include <cstring>
#include <algorithm>

namespace firmware::sensors {

bool NmeaParser::processByte(uint8_t byte) {
    char c = static_cast<char>(byte);

    if (c == '$') {
        // Start of new packet
        m_inPacket = true;
        m_bufferIndex = 0;
        m_buffer[m_bufferIndex++] = c;
        return false;
    }

    if (!m_inPacket) {
        return false;
    }

    // Store character
    if (m_bufferIndex < MAX_PACKET_SIZE) {
        m_buffer[m_bufferIndex++] = c;
    } else {
        // Buffer overflow: Reset state
        m_inPacket = false;
        m_bufferIndex = 0;
        return false;
    }

    // Check for end of packet (\n)
    if (c == '\n') {
        m_inPacket = false;
        // Construct string view excluding \r\n for easier processing
        // Minimal packet: "$A*CS\r\n" -> 7 chars
        if (m_bufferIndex >= 7) {
            std::string_view packet(m_buffer.data(), m_bufferIndex);

            // Remove CRLF or LF from end
            size_t trimLen = 0;
            if (packet.ends_with('\n')) trimLen++;
            if (packet.length() > trimLen && packet[packet.length() - trimLen - 1] == '\r') trimLen++;

            packet.remove_suffix(trimLen);

            if (validateChecksum(packet)) {
                parsePacket(packet);
                return true;
            }
        }
        m_bufferIndex = 0;
    }

    return false;
}

GpsData NmeaParser::getLatestData() const {
    // In a preemptive multitasking system, you might want a critical section here.
    // However, since GpsData is composed of primitives and we return by value,
    // the risk is primarily inconsistent fields (lat vs lon), not memory corruption.
    // For this RTOS implementation, atomic load/store or a mutex in the caller is preferred
    // if absolute strictness is required.
    return m_currentData;
}

bool NmeaParser::validateChecksum(std::string_view packet) const {
    // Packet format: $......*CS
    size_t starPos = packet.find_last_of('*');
    if (starPos == std::string_view::npos || starPos + 3 > packet.size()) {
        return false;
    }

    // 1. Parse received checksum
    std::string_view checksumStr = packet.substr(starPos + 1, 2);
    uint8_t receivedSum = 0;
    auto [ptr, ec] = std::from_chars(checksumStr.data(), checksumStr.data() + checksumStr.size(), receivedSum, 16);
    if (ec != std::errc()) {
        return false;
    }

    // 2. Calculate checksum (XOR bytes between '$' and '*')
    uint8_t calculatedSum = 0;
    for (size_t i = 1; i < starPos; i++) {
        calculatedSum ^= static_cast<uint8_t>(packet[i]);
    }

    return (receivedSum == calculatedSum);
}

void NmeaParser::parsePacket(std::string_view packet) {
    // packet: $ttsss,d1,d2....*CS
    // tt: Talker ID (GP, GN, GL) - WE IGNORE THIS to be multi-constellation compatible
    // sss: Sentence ID (GGA, RMC)

    size_t commaPos = packet.find(',');
    if (commaPos == std::string_view::npos || commaPos < 6) return;

    std::string_view addressField = packet.substr(1, commaPos - 1); // Skip '$'
    std::string_view body = packet.substr(commaPos + 1); // Content after first comma, including *CS

    // Extract Sentence ID (Last 3 chars of address field)
    if (addressField.size() < 3) return;
    std::string_view sentenceId = addressField.substr(addressField.size() - 3);

    if (sentenceId == "GGA") {
        parseGga(body);
    } else if (sentenceId == "RMC") {
        parseRmc(body);
    }
}

void NmeaParser::parseGga(std::string_view body) {
    // GGA Format: Time, Lat, N/S, Lon, E/W, FixQuality, Sats, HDOP, Alt, M, ...
    // Indexes:    0     1    2    3    4    5           6     7     8    9

    std::string_view latStr = getToken(body, 1);
    std::string_view nsStr = getToken(body, 2);
    std::string_view lonStr = getToken(body, 3);
    std::string_view ewStr = getToken(body, 4);
    std::string_view fixStr = getToken(body, 5);
    std::string_view satsStr = getToken(body, 6);
    std::string_view altStr = getToken(body, 8);

    int fixQuality = parseInt(fixStr);

    if (fixQuality > 0) {
        m_currentData.latitude = nmeaToDecimal(latStr, nsStr);
        m_currentData.longitude = nmeaToDecimal(lonStr, ewStr);
        m_currentData.altitude = parseFloat(altStr);
        m_currentData.satellites = static_cast<uint8_t>(parseInt(satsStr));
        m_currentData.fixType = static_cast<uint8_t>(fixQuality);
        m_currentData.valid = true;
    } else {
        m_currentData.valid = false;
        m_currentData.fixType = 0;
    }
}

void NmeaParser::parseRmc(std::string_view body) {
    // RMC Format: Time, Status, Lat, N/S, Lon, E/W, Speed, Course, Date...
    // Indexes:    0     1       2    3    4    5    6      7       8

    std::string_view statusStr = getToken(body, 1);
    std::string_view latStr = getToken(body, 2);
    std::string_view nsStr = getToken(body, 3);
    std::string_view lonStr = getToken(body, 4);
    std::string_view ewStr = getToken(body, 5);
    std::string_view speedStr = getToken(body, 6);
    std::string_view courseStr = getToken(body, 7);

    if (statusStr.starts_with('A')) { // A = Active/Valid
        m_currentData.latitude = nmeaToDecimal(latStr, nsStr);
        m_currentData.longitude = nmeaToDecimal(lonStr, ewStr);

        // NMEA speed is in Knots. Convert to m/s.
        // 1 Knot = 0.514444 m/s
        m_currentData.speed = parseFloat(speedStr) * 0.514444f;
        m_currentData.course = parseFloat(courseStr);
        m_currentData.valid = true;
        // RMC implies at least a basic fix
        if (m_currentData.fixType == 0) m_currentData.fixType = 1;
    }
}

// --- Helpers ---

std::string_view NmeaParser::getToken(std::string_view str, size_t index) const {
    size_t start = 0;
    size_t end = str.find(',');

    size_t currentIdx = 0;

    while (currentIdx < index) {
        if (end == std::string_view::npos) return {}; // Token not found
        start = end + 1;
        end = str.find(',', start);
        currentIdx++;
    }

    // Handle the checksum separator '*' if it appears in the last token
    if (end == std::string_view::npos) {
        end = str.find('*');
        if (end == std::string_view::npos) end = str.size();
    } else {
        // If the token contains '*', truncate it there
        size_t star = str.substr(start, end - start).find('*');
        if (star != std::string_view::npos) {
            end = start + star;
        }
    }

    return str.substr(start, end - start);
}

double NmeaParser::nmeaToDecimal(std::string_view val, std::string_view quadrant) {
    if (val.size() < 4) return 0.0;

    // NMEA format: dddmm.mmmm (Longitude) or ddmm.mmmm (Latitude)
    // We find the decimal point to separate degrees and minutes safely
    size_t decimalPos = val.find('.');
    if (decimalPos == std::string_view::npos || decimalPos < 2) return 0.0;

    // Degrees are everything before the last 2 digits of the integer part
    // e.g., 4715.123 -> Degrees: 47, Minutes: 15.123
    size_t degreesLen = decimalPos - 2;

    std::string_view degStr = val.substr(0, degreesLen);
    std::string_view minStr = val.substr(degreesLen);

    double deg = parseDouble(degStr);
    double min = parseDouble(minStr);

    double decimal = deg + (min / 60.0);

    if (quadrant.starts_with('S') || quadrant.starts_with('W')) {
        decimal = -decimal;
    }

    return decimal;
}

float NmeaParser::parseFloat(std::string_view str) const {
    if (str.empty()) return 0.0f;
    float result = 0.0f;
    // std::from_chars is preferred for embedded C++17/23 (no heap, no locale)
    std::from_chars(str.data(), str.data() + str.size(), result);
    return result;
}

double NmeaParser::parseDouble(std::string_view str) const {
    if (str.empty()) return 0.0;
    double result = 0.0;
    std::from_chars(str.data(), str.data() + str.size(), result);
    return result;
}

int NmeaParser::parseInt(std::string_view str) const {
    if (str.empty()) return 0;
    int result = 0;
    std::from_chars(str.data(), str.data() + str.size(), result);
    return result;
}

} // namespace firmware::sensors
