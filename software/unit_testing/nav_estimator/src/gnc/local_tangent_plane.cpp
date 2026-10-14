/**
 * @file local_tangent_plane.cpp
 * @brief WGS-84 geodetic <-> local NED conversion.
 */

#include "local_tangent_plane.hpp"
#include <cmath>

namespace gnc::ins {

namespace {
constexpr double WGS84_A = 6378137.0;                 ///< semi-major axis [m]
constexpr double WGS84_F = 1.0 / 298.257223563;       ///< flattening
constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);///< first eccentricity squared
constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;
}

Eigen::Vector3d LocalTangentPlane::geodeticToEcef(double latDeg, double lonDeg, double altM)
{
    const double lat = latDeg * DEG2RAD;
    const double lon = lonDeg * DEG2RAD;
    const double sLat = std::sin(lat), cLat = std::cos(lat);
    const double sLon = std::sin(lon), cLon = std::cos(lon);
    const double n = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sLat * sLat);
    return { (n + altM) * cLat * cLon,
             (n + altM) * cLat * sLon,
             (n * (1.0 - WGS84_E2) + altM) * sLat };
}

Lla LocalTangentPlane::ecefToGeodetic(const Eigen::Vector3d& ecef)
{
    // Iterative solution; converges to sub-millimetre in a handful of steps.
    const double x = ecef.x(), y = ecef.y(), z = ecef.z();
    const double p = std::sqrt(x * x + y * y);
    const double lon = std::atan2(y, x);

    double lat = std::atan2(z, p * (1.0 - WGS84_E2));
    double alt = 0.0;
    for (int i = 0; i < 6; ++i) {
        const double sLat = std::sin(lat);
        const double n = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sLat * sLat);
        alt = (std::abs(std::cos(lat)) > 1e-6) ? (p / std::cos(lat) - n)
                                               : (std::abs(z) / std::abs(sLat) - n * (1.0 - WGS84_E2));
        lat = std::atan2(z, p * (1.0 - WGS84_E2 * n / (n + alt)));
    }
    return { lat * RAD2DEG, lon * RAD2DEG, alt };
}

Eigen::Matrix3d LocalTangentPlane::ecefToNedRotation(double latDeg, double lonDeg)
{
    const double lat = latDeg * DEG2RAD;
    const double lon = lonDeg * DEG2RAD;
    const double sLat = std::sin(lat), cLat = std::cos(lat);
    const double sLon = std::sin(lon), cLon = std::cos(lon);
    Eigen::Matrix3d r;
    r << -sLat * cLon, -sLat * sLon,  cLat,
         -sLon,         cLon,         0.0,
         -cLat * cLon, -cLat * sLon, -sLat;
    return r;
}

void LocalTangentPlane::setOrigin(double latDeg, double lonDeg, double altM)
{
    m_origin = { latDeg, lonDeg, altM };
    m_originEcef = geodeticToEcef(latDeg, lonDeg, altM);
    m_rEcefToNed = ecefToNedRotation(latDeg, lonDeg);
    m_hasOrigin = true;
}

Eigen::Vector3d LocalTangentPlane::llaToNedD(double latDeg, double lonDeg, double altM) const
{
    return m_rEcefToNed * (geodeticToEcef(latDeg, lonDeg, altM) - m_originEcef);
}

Eigen::Vector3f LocalTangentPlane::llaToNed(double latDeg, double lonDeg, double altM) const
{
    return llaToNedD(latDeg, lonDeg, altM).cast<float>();
}

Lla LocalTangentPlane::nedToLla(const Eigen::Vector3f& ned) const
{
    const Eigen::Vector3d ecef = m_originEcef + m_rEcefToNed.transpose() * ned.cast<double>();
    return ecefToGeodetic(ecef);
}

Eigen::Matrix3f LocalTangentPlane::recentre(const Eigen::Vector3f& deltaNed)
{
    const Eigen::Matrix3d rOld = m_rEcefToNed;
    const Eigen::Vector3d ecefNew = m_originEcef + rOld.transpose() * deltaNed.cast<double>();
    const Lla lla = ecefToGeodetic(ecefNew);

    setOrigin(lla.latDeg, lla.lonDeg, lla.altM);

    // v_new = R_ecef2ned_new * R_ecef2ned_old^T * v_old
    const Eigen::Matrix3d rNewFromOld = m_rEcefToNed * rOld.transpose();
    return rNewFromOld.cast<float>();
}

} // namespace gnc::ins
