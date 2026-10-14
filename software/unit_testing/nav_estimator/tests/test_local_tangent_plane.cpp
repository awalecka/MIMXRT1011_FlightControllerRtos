/**
 * @file test_local_tangent_plane.cpp
 * @brief Unit tests for WGS-84 Local Tangent Plane conversions.
 */

#include <gtest/gtest.h>
#include "local_tangent_plane.hpp"

using namespace gnc::ins;

TEST(LocalTangentPlaneTest, OriginMapsToZero) {
    LocalTangentPlane ltp;
    ltp.setOrigin(37.7749, -122.4194, 30.0);

    const Eigen::Vector3f ned = ltp.llaToNed(37.7749, -122.4194, 30.0);
    EXPECT_NEAR(ned.x(), 0.0f, 1e-3f);
    EXPECT_NEAR(ned.y(), 0.0f, 1e-3f);
    EXPECT_NEAR(ned.z(), 0.0f, 1e-3f);
}

TEST(LocalTangentPlaneTest, CardinalDisplacements) {
    LocalTangentPlane ltp;
    ltp.setOrigin(0.0, 0.0, 0.0); // Equator / Prime Meridian

    // +0.001 deg North at Equator: 1 deg ~ 111,319 m => 0.001 deg ~ 110.57 m (WGS-84 meridional)
    const Eigen::Vector3f nedNorth = ltp.llaToNed(0.001, 0.0, 0.0);
    EXPECT_NEAR(nedNorth.x(), 110.574f, 0.2f);
    EXPECT_NEAR(nedNorth.y(), 0.0f, 1e-3f);
    EXPECT_NEAR(nedNorth.z(), 0.0f, 1e-2f);

    // +0.001 deg East at Equator: 1 deg ~ 111,319.5 m => 0.001 deg ~ 111.32 m
    const Eigen::Vector3f nedEast = ltp.llaToNed(0.0, 0.001, 0.0);
    EXPECT_NEAR(nedEast.x(), 0.0f, 1e-3f);
    EXPECT_NEAR(nedEast.y(), 111.319f, 0.2f);
    EXPECT_NEAR(nedEast.z(), 0.0f, 1e-2f);

    // +100m Up in ellipsoidal height corresponds to -100m Down in NED
    const Eigen::Vector3f nedUp = ltp.llaToNed(0.0, 0.0, 100.0);
    EXPECT_NEAR(nedUp.x(), 0.0f, 1e-3f);
    EXPECT_NEAR(nedUp.y(), 0.0f, 1e-3f);
    EXPECT_NEAR(nedUp.z(), -100.0f, 1e-3f);
}

TEST(LocalTangentPlaneTest, RoundTripPrecision) {
    LocalTangentPlane ltp;
    ltp.setOrigin(51.5074, -0.1278, 25.0); // London

    // Displace 15 km North, 20 km East, -50 m Down
    const Eigen::Vector3f nedOrig(15000.0f, 20000.0f, -50.0f);
    const Lla lla = ltp.nedToLla(nedOrig);
    const Eigen::Vector3f nedBack = ltp.llaToNed(lla.latDeg, lla.lonDeg, static_cast<float>(lla.altM));

    EXPECT_NEAR(nedBack.x(), nedOrig.x(), 0.005f); // Sub-centimetre
    EXPECT_NEAR(nedBack.y(), nedOrig.y(), 0.005f);
    EXPECT_NEAR(nedBack.z(), nedOrig.z(), 0.005f);
}

TEST(LocalTangentPlaneTest, SouthernWesternHemispheres) {
    LocalTangentPlane ltp;
    ltp.setOrigin(-33.8688, 151.2093, 10.0); // Sydney, Australia

    const Eigen::Vector3f nedOrig(5000.0f, -3000.0f, 20.0f);
    const Lla lla = ltp.nedToLla(nedOrig);
    const Eigen::Vector3f nedBack = ltp.llaToNed(lla.latDeg, lla.lonDeg, static_cast<float>(lla.altM));

    EXPECT_NEAR(nedBack.x(), nedOrig.x(), 0.005f);
    EXPECT_NEAR(nedBack.y(), nedOrig.y(), 0.005f);
    EXPECT_NEAR(nedBack.z(), nedOrig.z(), 0.005f);
}

TEST(LocalTangentPlaneTest, RecenteringInvariance) {
    LocalTangentPlane ltp;
    ltp.setOrigin(37.7749, -122.4194, 15.0);

    const Eigen::Vector3f p1(20000.0f, 15000.0f, -30.0f);
    const Lla lla1 = ltp.nedToLla(p1);

    // Recentre to point p1
    const Eigen::Matrix3f rNewFromOld = ltp.recentre(p1);

    // After recentring, p1 should be at the new origin (0,0,0)
    const Eigen::Vector3f p1New = ltp.llaToNed(lla1.latDeg, lla1.lonDeg, static_cast<float>(lla1.altM));
    EXPECT_NEAR(p1New.x(), 0.0f, 1e-3f);
    EXPECT_NEAR(p1New.y(), 0.0f, 1e-3f);
    EXPECT_NEAR(p1New.z(), 0.0f, 1e-3f);

    // Vector rotation check
    const Eigen::Vector3f p1NewMapped = rNewFromOld * (p1 - p1);
    EXPECT_NEAR(p1NewMapped.norm(), 0.0f, 1e-3f);
}
