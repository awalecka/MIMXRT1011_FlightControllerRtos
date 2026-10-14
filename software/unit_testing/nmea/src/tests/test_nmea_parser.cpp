/**
 * @file test_nmea_parser.cpp
 * @brief GoogleTest unit tests for the embedded NmeaParser.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>
#include <iomanip>
#include <sstream>
#include "nmea.h" 

using namespace firmware::sensors;

class NmeaParserTest : public ::testing::Test {
protected:
    NmeaParser parser;

    /**
     * @brief Helper to feed a string into the parser byte-by-byte.
     * @return true if at least one packet was successfully completed.
     */
    bool feedParser(std::string_view packet) {
        bool packetComplete = false;
        for (char c : packet) {
            if (parser.processByte(static_cast<uint8_t>(c))) {
                packetComplete = true;
            }
        }
        return packetComplete;
    }

    /**
     * @brief Helper to construct a valid NMEA packet with checksum.
     * @param content The payload inside the packet (e.g., "GPGGA,123...").
     * @return Full packet string: "$CONTENT*CS\r\n"
     */
    std::string makePacket(std::string_view content) {
        uint8_t checksum = 0;
        for (char c : content) {
            checksum ^= static_cast<uint8_t>(c);
        }

        std::stringstream ss;
        ss << "$" << content << "*" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(checksum) << "\r\n";
        return ss.str();
    }
};

/**
 * @brief Validates GNRMC parsing (Time, Lat, Lon, Speed, Course).
 */
TEST_F(NmeaParserTest, GNRMC_Valid) {
    // Standard valid packet
    std::string packet = makePacket("GNRMC,220516.000,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");

    EXPECT_TRUE(feedParser(packet));

    GpsData data = parser.getLatestData();
    EXPECT_TRUE(data.valid);
    EXPECT_NEAR(data.latitude, 51.563666, 0.0001);
    EXPECT_NEAR(data.longitude, -0.704, 0.0001);
    EXPECT_NEAR(data.speed, 173.8 * 0.514444, 0.01); // Knots to m/s
    EXPECT_NEAR(data.course, 231.8, 0.01);
    EXPECT_GE(data.fixType, 1);
}

/**
 * @brief Validates GNGGA parsing (Fix Quality, Altitude, Satellites).
 */
TEST_F(NmeaParserTest, GNGGA_Valid) {
    std::string packet = makePacket("GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,");

    EXPECT_TRUE(feedParser(packet));

    GpsData data = parser.getLatestData();
    EXPECT_TRUE(data.valid);
    EXPECT_NEAR(data.latitude, 44.069006, 0.00001);
    EXPECT_NEAR(data.altitude, 1113.0, 0.01);
    EXPECT_EQ(data.satellites, 12);
    EXPECT_EQ(data.fixType, 1);
}

/**
 * @brief Ensures the parser handles different Talker IDs (GP, GL, GN).
 */
TEST_F(NmeaParserTest, MultiConstellation) {
    // $GPGGA (GPS only)
    std::string packetGPS = makePacket("GPGGA,001043.00,4404.14036,N,12118.85961,W,1,05,0.98,100.0,M,-21.3,M,,");
    EXPECT_TRUE(feedParser(packetGPS));
    EXPECT_EQ(parser.getLatestData().satellites, 5);

    // $GLGGA (GLONASS only)
    std::string packetGLO = makePacket("GLGGA,001043.00,4404.14036,N,12118.85961,W,1,07,0.98,200.0,M,-21.3,M,,");
    EXPECT_TRUE(feedParser(packetGLO));
    EXPECT_EQ(parser.getLatestData().satellites, 7);
    EXPECT_NEAR(parser.getLatestData().altitude, 200.0, 0.1);
}

/**
 * @brief Ensures packets with invalid checksums are rejected.
 */
TEST_F(NmeaParserTest, Checksum_Fail) {
    // Manually construct bad checksum packet
    std::string packet = "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,*00\r\n";

    EXPECT_FALSE(feedParser(packet));
    EXPECT_FALSE(parser.getLatestData().valid);
}

/**
 * @brief Simulates fragmented data arrival.
 */
TEST_F(NmeaParserTest, Fragmentation) {
    std::string full = makePacket("GNRMC,220516.000,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");

    // Split into 3 chunks
    std::string part1 = full.substr(0, 10);
    std::string part2 = full.substr(10, 20);
    std::string part3 = full.substr(30);

    EXPECT_FALSE(feedParser(part1));
    EXPECT_FALSE(feedParser(part2));
    EXPECT_TRUE(feedParser(part3));

    EXPECT_TRUE(parser.getLatestData().valid);
}

/**
 * @brief Tests recovery from buffer overflow conditions.
 * Sends >85 bytes of garbage, then a valid packet.
 */
TEST_F(NmeaParserTest, BufferOverflow) {
    // 1. Fill buffer with garbage (> MAX_PACKET_SIZE 85)
    std::string garbage(90, 'A');
    EXPECT_FALSE(feedParser(garbage));

    // 2. Send valid packet
    std::string valid = makePacket("GNRMC,220516.000,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");
    EXPECT_TRUE(feedParser(valid));

    // 3. Should recover
    EXPECT_TRUE(parser.getLatestData().valid);
}

/**
 * @brief Tests resynchronization when a packet is interrupted by a new start character '$'.
 */
TEST_F(NmeaParserTest, MidPacketRestart) {
    // 1. Send incomplete packet start
    std::string incomplete = "$GPGGA,12345,";
    EXPECT_FALSE(feedParser(incomplete));

    // 2. Immediately send a full valid packet (Simulating lost bytes/restart)
    std::string valid = makePacket("GNRMC,220516.000,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");
    EXPECT_TRUE(feedParser(valid));

    EXPECT_TRUE(parser.getLatestData().valid);
    EXPECT_NEAR(parser.getLatestData().latitude, 51.563666, 0.0001);
}

/**
 * @brief Tests GGA logic when Fix Quality is 0 (Invalid).
 */
TEST_F(NmeaParserTest, GGA_NoFix) {
    // Fix Quality = 0
    std::string packet = makePacket("GNGGA,001043.00,4404.14036,N,12118.85961,W,0,00,99.99,1113.0,M,-21.3,M,,");

    EXPECT_TRUE(feedParser(packet));

    auto data = parser.getLatestData();
    EXPECT_FALSE(data.valid);
    EXPECT_EQ(data.fixType, 0);
}

/**
 * @brief Tests RMC logic when Status is 'V' (Void).
 */
TEST_F(NmeaParserTest, RMC_VoidStatus) {
    // Status = V (Void)
    std::string packet = makePacket("GNRMC,220516.000,V,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");

    EXPECT_TRUE(feedParser(packet));

    // Data should NOT be marked valid
    EXPECT_FALSE(parser.getLatestData().valid); // Or implementation dependent: valid=false
}

/**
 * @brief Tests parsing with empty fields to ensure no crashes.
 */
TEST_F(NmeaParserTest, EmptyFields) {
    // Many empty fields, but valid syntax
    std::string packet = makePacket("GPGGA,123519,,,,,0,00,,,M,,M,,");

    EXPECT_TRUE(feedParser(packet));

    // Should simply result in invalid data, but NO crash
    EXPECT_FALSE(parser.getLatestData().valid);
}

/**
 * @brief Tests robustness against short/malformed coordinate strings.
 */
TEST_F(NmeaParserTest, MalformedCoordinates) {
    // Lat "12" is too short for "ddmm.mmmm" format (needs at least 4 chars usually, or safer logic)
    // The parser check `if (val.size() < 4) return 0.0;` should handle this.
    std::string packet = makePacket("GNRMC,220516.000,A,12,N,1,W,173.8,231.8,130694,004.2,W");

    EXPECT_TRUE(feedParser(packet));

    GpsData data = parser.getLatestData();
    EXPECT_TRUE(data.valid); // It is a "valid" packet structure
    EXPECT_EQ(data.latitude, 0.0); // Should fallback to 0.0 safely
    EXPECT_EQ(data.longitude, 0.0);
}

/**
 * @brief explicit check for Southern and Western Hemisphere negative signs.
 */
TEST_F(NmeaParserTest, HemisphereLogic) {
    // 1000.00 S -> -10 deg
    // 02000.00 W -> -20 deg
    std::string packet = makePacket("GNRMC,220516.000,A,1000.00,S,02000.00,W,10.0,0.0,130694,004.2,W");

    EXPECT_TRUE(feedParser(packet));

    GpsData data = parser.getLatestData();
    EXPECT_NEAR(data.latitude, -10.0, 0.0001);
    EXPECT_NEAR(data.longitude, -20.0, 0.0001);
}
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


// ============================================================================
// Extended fields: HDOP, UTC time, epochs, VTG, GST
// ============================================================================

TEST_F(NmeaParserTest, GGA_HdopAndTime) {
    EXPECT_TRUE(feedParser(makePacket("GNGGA,123519.250,4807.038,N,01131.000,E,1,08,0.90,545.4,M,46.9,M,,")));
    GpsData d = parser.getLatestData();
    EXPECT_NEAR(d.hdop, 0.90f, 1e-4f);
    EXPECT_EQ(d.utcTimeMs, ((12u * 60u + 35u) * 60u + 19u) * 1000u + 250u);
    EXPECT_EQ(d.epochId, 1u);
    EXPECT_EQ(d.posEpochId, 1u);
}

TEST_F(NmeaParserTest, VTG_Valid) {
    EXPECT_TRUE(feedParser(makePacket("GNVTG,054.7,T,034.4,M,005.5,N,010.2,K,A")));
    GpsData d = parser.getLatestData();
    EXPECT_TRUE(d.hasVelocity);
    EXPECT_NEAR(d.speed, 10.2f / 3.6f, 1e-3f);
    EXPECT_NEAR(d.course, 54.7f, 1e-3f);
}

TEST_F(NmeaParserTest, VTG_ModeN_Invalid) {
    EXPECT_TRUE(feedParser(makePacket("GNVTG,054.7,T,034.4,M,005.5,N,010.2,K,N")));
    GpsData d = parser.getLatestData();
    EXPECT_FALSE(d.hasVelocity);
    EXPECT_EQ(d.velEpochId, 0u);
}

TEST_F(NmeaParserTest, VTG_EmptyCourseKeepsPrevious) {
    feedParser(makePacket("GNVTG,120.0,T,,M,005.5,N,010.0,K,A"));
    feedParser(makePacket("GNVTG,,T,,M,000.0,N,000.0,K,A"));
    GpsData d = parser.getLatestData();
    EXPECT_TRUE(d.hasVelocity);
    EXPECT_NEAR(d.speed, 0.0f, 1e-6f);
    EXPECT_NEAR(d.course, 120.0f, 1e-3f);
}

TEST_F(NmeaParserTest, GST_Valid) {
    EXPECT_TRUE(feedParser(makePacket("GNGST,082356.00,1.8,,,,1.7,1.3,2.2")));
    GpsData d = parser.getLatestData();
    EXPECT_TRUE(d.hasGst);
    EXPECT_NEAR(d.sigmaLat, 1.7f, 1e-4f);
    EXPECT_NEAR(d.sigmaLon, 1.3f, 1e-4f);
    EXPECT_NEAR(d.sigmaAlt, 2.2f, 1e-4f);
    EXPECT_EQ(d.gstTimeMs, ((8u * 60u + 23u) * 60u + 56u) * 1000u);
}

TEST_F(NmeaParserTest, GST_EmptySigmasIgnored) {
    feedParser(makePacket("GNGST,082356.00,1.8,,,,,,"));
    EXPECT_FALSE(parser.getLatestData().hasGst);
}

TEST_F(NmeaParserTest, EpochId_IncrementsOncePerTime) {
    // GGA + RMC + VTG + GST of one epoch, then a second epoch.
    feedParser(makePacket("GNGGA,100000.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,"));
    feedParser(makePacket("GNRMC,100000.00,A,4404.14036,N,12118.85961,W,10.0,90.0,130694,,"));
    feedParser(makePacket("GNVTG,90.0,T,,M,10.0,N,18.5,K,A"));
    feedParser(makePacket("GNGST,100000.00,1.0,,,,1.5,1.5,3.0"));
    GpsData d = parser.getLatestData();
    EXPECT_EQ(d.epochId, 1u);
    EXPECT_EQ(d.posEpochId, 1u) << "GGA and RMC of one epoch must count once";
    EXPECT_EQ(d.velEpochId, 1u) << "RMC and VTG of one epoch must count once";

    feedParser(makePacket("GNGGA,100000.20,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,"));
    feedParser(makePacket("GNRMC,100000.20,A,4404.14036,N,12118.85961,W,10.0,90.0,130694,,"));
    d = parser.getLatestData();
    EXPECT_EQ(d.epochId, 2u);
    EXPECT_EQ(d.posEpochId, 2u);
    EXPECT_EQ(d.velEpochId, 2u);
}

TEST_F(NmeaParserTest, VelocityFromVtgWhenRmcVoid) {
    // RMC void (no velocity), VTG valid for the same epoch: velocity counts.
    feedParser(makePacket("GNGGA,100000.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,"));
    feedParser(makePacket("GNRMC,100000.00,V,,,,,,,130694,,"));
    feedParser(makePacket("GNVTG,90.0,T,,M,10.0,N,18.5,K,A"));
    GpsData d = parser.getLatestData();
    EXPECT_EQ(d.velEpochId, 1u);
    EXPECT_TRUE(d.hasVelocity);
}

TEST_F(NmeaParserTest, UtcTime_Midnight_Wrap) {
    feedParser(makePacket("GNGGA,235959.80,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,"));
    EXPECT_EQ(parser.getLatestData().utcTimeMs, 86399800u);
    feedParser(makePacket("GNGGA,000000.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,"));
    GpsData d = parser.getLatestData();
    EXPECT_EQ(d.utcTimeMs, 0u);
    EXPECT_EQ(d.epochId, 2u) << "Midnight rollover is a new epoch";
}

TEST_F(NmeaParserTest, FirstEpochAtZeroTimeCounts) {
    feedParser(makePacket("GNGGA,000000.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M,,"));
    EXPECT_EQ(parser.getLatestData().posEpochId, 1u);
}

TEST_F(NmeaParserTest, NoFixGgaDoesNotAdvancePosEpoch) {
    feedParser(makePacket("GNGGA,100000.00,,,,,0,00,99.99,,,,,,"));
    GpsData d = parser.getLatestData();
    EXPECT_EQ(d.posEpochId, 0u);
    EXPECT_EQ(d.epochId, 1u);
}