/**
 * @file test_nmea_parser.cpp
 * @brief Host-based unit tests for the embedded NmeaParser.
 * * Compile with:
 * g++ -std=c++20 -I../source/sensors test_nmea_parser.cpp ../source/sensors/nmea_parser.cpp -o run_tests
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <cassert>
#include "nmea.h"

using namespace firmware::sensors;

// --- Test Framework Helpers ---
#define ASSERT_TRUE(condition) \
    if (!(condition)) { \
        std::cerr << "FAILED: " << #condition << " at line " << __LINE__ << std::endl; \
        return false; \
    }

#define ASSERT_NEAR(val1, val2, abs_error) \
    if (std::abs((val1) - (val2)) > (abs_error)) { \
        std::cerr << "FAILED: " << #val1 << " (" << (val1) << ") != " << #val2 << " (" << (val2) << ") at line " << __LINE__ << std::endl; \
        return false; \
    }

// Helper to feed string to parser byte-by-byte
bool feedParser(NmeaParser& parser, std::string_view packet) {
    bool packetComplete = false;
    for (char c : packet) {
        if (parser.processByte(static_cast<uint8_t>(c))) {
            packetComplete = true;
        }
    }
    return packetComplete;
}

// --- Test Cases ---

/**
 * @brief Validates GNRMC parsing (Time, Lat, Lon, Speed, Course).
 * Packet: $GNRMC,220516.000,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
 */
bool Test_GNRMC_Valid() {
    NmeaParser parser;
    // Note: CRLF (\r\n) is standard NMEA terminator
    const char* packet = "$GNRMC,220516.000,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r\n";

    ASSERT_TRUE(feedParser(parser, packet));

    GpsData data = parser.getLatestData();
    ASSERT_TRUE(data.valid);

    // Lat: 5133.82 N -> 51 degrees, 33.82 minutes -> 51 + (33.82/60) = 51.563666...
    ASSERT_NEAR(data.latitude, 51.563666, 0.0001);

    // Lon: 00042.24 W -> 0 degrees, 42.24 minutes -> -(0 + 42.24/60) = -0.704
    ASSERT_NEAR(data.longitude, -0.704, 0.0001);

    // Speed: 173.8 Knots -> m/s. 1 knot = 0.514444 m/s
    ASSERT_NEAR(data.speed, 173.8 * 0.514444, 0.01);

    // Course: 231.8
    ASSERT_NEAR(data.course, 231.8, 0.01);

    // Fix Type should be at least 1 (GPS Fix) since Status was 'A'
    ASSERT_TRUE(data.fixType >= 1);

    return true;
}

/**
 * @brief Validates GNGGA parsing (Fix Quality, Altitude, Satellites).
 * Packet: $GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,*47
 */
bool Test_GNGGA_Valid() {
    NmeaParser parser;
    const char* packet = "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,*47\r\n";

    ASSERT_TRUE(feedParser(parser, packet));

    GpsData data = parser.getLatestData();
    ASSERT_TRUE(data.valid);

    // Lat: 4404.14036 N -> 44 + 4.14036/60
    ASSERT_NEAR(data.latitude, 44.069006, 0.00001);

    // Alt: 1113.0
    ASSERT_NEAR(data.altitude, 1113.0, 0.01);

    // Sats: 12
    ASSERT_TRUE(data.satellites == 12);

    // Fix Type: 1
    ASSERT_TRUE(data.fixType == 1);

    return true;
}

/**
 * @brief Ensures the parser handles different Talker IDs (GP, GL, GN) by ignoring them
 * and matching only the Sentence ID (GGA).
 */
bool Test_MultiConstellation() {
    NmeaParser parser;

    // $GPGGA (GPS only)
    const char* packetGPS = "$GPGGA,001043.00,4404.14036,N,12118.85961,W,1,05,0.98,100.0,M,-21.3,M,,*5C\r\n";
    ASSERT_TRUE(feedParser(parser, packetGPS));
    ASSERT_TRUE(parser.getLatestData().satellites == 5);

    // $GLGGA (GLONASS only) - Same pos, different sats/alt to verify update
    const char* packetGLO = "$GLGGA,001043.00,4404.14036,N,12118.85961,W,1,07,0.98,200.0,M,-21.3,M,,*5D\r\n";
    ASSERT_TRUE(feedParser(parser, packetGLO));
    ASSERT_TRUE(parser.getLatestData().satellites == 7);
    ASSERT_NEAR(parser.getLatestData().altitude, 200.0, 0.1);

    return true;
}

/**
 * @brief Ensures packets with invalid checksums are rejected.
 */
bool Test_Checksum_Fail() {
    NmeaParser parser;
    // Modified checksum from *47 to *46
    const char* packet = "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,*46\r\n";

    // Should return false (packet not accepted)
    ASSERT_TRUE(feedParser(parser, packet) == false);

    // Data should remain invalid/default
    ASSERT_TRUE(parser.getLatestData().valid == false);

    return true;
}

/**
 * @brief Simulates receiving data in fragmented chunks (typical of UART/DMA ring buffers).
 */
bool Test_Fragmentation() {
    NmeaParser parser;
    std::string part1 = "$GNRMC,220516.000,A,5133.";
    std::string part2 = "82,N,00042.24,W,173.8,231.";
    std::string part3 = "8,130694,004.2,W*70\r\n";

    ASSERT_TRUE(feedParser(parser, part1) == false); // Not done
    ASSERT_TRUE(feedParser(parser, part2) == false); // Not done
    ASSERT_TRUE(feedParser(parser, part3) == true);  // Done

    ASSERT_TRUE(parser.getLatestData().valid);
    ASSERT_NEAR(parser.getLatestData().latitude, 51.563666, 0.0001);

    return true;
}

/**
 * @brief Checks behavior with noise/garbage before the packet.
 */
bool Test_Noise_Recovery() {
    NmeaParser parser;
    std::string noise = "98234@#%@#$%$GNRMC,220516.000,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r\n";

    ASSERT_TRUE(feedParser(parser, noise));
    ASSERT_TRUE(parser.getLatestData().valid);

    return true;
}

int main() {
    std::cout << "Running NMEA Parser Unit Tests..." << std::endl;

    bool allPassed = true;

    if (Test_GNRMC_Valid()) std::cout << "[PASS] Test_GNRMC_Valid" << std::endl;
    else { std::cout << "[FAIL] Test_GNRMC_Valid" << std::endl; allPassed = false; }

    if (Test_GNGGA_Valid()) std::cout << "[PASS] Test_GNGGA_Valid" << std::endl;
    else { std::cout << "[FAIL] Test_GNGGA_Valid" << std::endl; allPassed = false; }

    if (Test_MultiConstellation()) std::cout << "[PASS] Test_MultiConstellation" << std::endl;
    else { std::cout << "[FAIL] Test_MultiConstellation" << std::endl; allPassed = false; }

    if (Test_Checksum_Fail()) std::cout << "[PASS] Test_Checksum_Fail" << std::endl;
    else { std::cout << "[FAIL] Test_Checksum_Fail" << std::endl; allPassed = false; }

    if (Test_Fragmentation()) std::cout << "[PASS] Test_Fragmentation" << std::endl;
    else { std::cout << "[FAIL] Test_Fragmentation" << std::endl; allPassed = false; }

    if (Test_Noise_Recovery()) std::cout << "[PASS] Test_Noise_Recovery" << std::endl;
    else { std::cout << "[FAIL] Test_Noise_Recovery" << std::endl; allPassed = false; }

    if (allPassed) {
        std::cout << "\nAll Tests Passed Successfully!" << std::endl;
        return 0;
    } else {
        std::cout << "\nSome Tests Failed." << std::endl;
        return -1;
    }
}
