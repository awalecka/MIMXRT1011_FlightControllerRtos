/**
 * @file local_tangent_plane.hpp
 * @brief WGS-84 geodetic <-> local NED conversion (double precision internally).
 *
 * The filter works in float32 local NED metres. Latitude/longitude need double
 * precision (1e-7 deg ~ 1 cm), so the conversion is done in double via ECEF
 * once per GPS fix and only the small relative NED vector is cast to float.
 *
 * Re-centring: when the vehicle has moved far from the origin the NED axes at
 * the origin no longer match local North/East/Down. recentre() moves the origin
 * and returns the rotation that maps old-NED vectors to new-NED vectors so the
 * filter can rebase its position, velocity and attitude consistently.
 */

#ifndef LOCAL_TANGENT_PLANE_HPP
#define LOCAL_TANGENT_PLANE_HPP

#include <Eigen>

namespace gnc::ins {

struct Lla {
    double latDeg = 0.0;
    double lonDeg = 0.0;
    double altM = 0.0;   ///< height above the WGS-84 ellipsoid as supplied by the caller
};

class LocalTangentPlane {
public:
    /** @brief Set the origin; NED (0,0,0) maps to this point. */
    void setOrigin(double latDeg, double lonDeg, double altM);

    [[nodiscard]] bool hasOrigin() const { return m_hasOrigin; }
    [[nodiscard]] Lla origin() const { return m_origin; }

    /** @brief Geodetic -> NED relative to the origin (double precision). */
    [[nodiscard]] Eigen::Vector3d llaToNedD(double latDeg, double lonDeg, double altM) const;

    /** @brief Geodetic -> NED relative to the origin, cast to float. */
    [[nodiscard]] Eigen::Vector3f llaToNed(double latDeg, double lonDeg, double altM) const;

    /** @brief NED relative to the origin -> geodetic. */
    [[nodiscard]] Lla nedToLla(const Eigen::Vector3f& ned) const;

    /**
     * @brief Move the origin to the point currently at `deltaNed` (old NED).
     * @return Rotation R such that  v_new = R * v_old  for free vectors, and
     *         p_new = R * (p_old - deltaNed) for positions.
     */
    Eigen::Matrix3f recentre(const Eigen::Vector3f& deltaNed);

private:
    static Eigen::Vector3d geodeticToEcef(double latDeg, double lonDeg, double altM);
    static Lla ecefToGeodetic(const Eigen::Vector3d& ecef);
    static Eigen::Matrix3d ecefToNedRotation(double latDeg, double lonDeg);

    bool m_hasOrigin = false;
    Lla m_origin;
    Eigen::Vector3d m_originEcef = Eigen::Vector3d::Zero();
    Eigen::Matrix3d m_rEcefToNed = Eigen::Matrix3d::Identity();
};

} // namespace gnc::ins

#endif // LOCAL_TANGENT_PLANE_HPP
