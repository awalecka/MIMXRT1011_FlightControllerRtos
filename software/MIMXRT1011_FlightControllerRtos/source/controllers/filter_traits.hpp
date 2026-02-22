/**
 * @file filter_traits.hpp
 * @brief Per-filter traits: Config construction and API adaptation shims.
 *
 * Why traits?
 * -----------
 * The two filters have slightly different Config structs and diverge on one
 * output method (UKF exposes getState() returning a 6D error vector; MEKF
 * exposes getBias() returning a 3D bias Vector3). Rather than forcing both
 * filters to adopt a common Config or changing either filter's own header,
 * FilterTraits<T> is specialised for each filter to:
 *
 *   1. Construct a correctly-typed Config from the single canonical set of
 *      tuning parameters that FlightController knows about (see below).
 *
 *   2. Provide a uniform getBias(filter) free function so FlightController
 *      can call FilterTraits<Filter>::getBias(filter_) without branching on
 *      filter type.
 *
 *   3. Provide getCovarianceFaultCount(filter) as a uniform free function.
 *      AttitudeUkf tracks this via a file-static counter; the trait wraps
 *      the UKF in a thin adapter that exposes the same diagnostic API as the
 *      MEKF without modifying the UKF's own interface.
 *
 * Adding a new filter
 * -------------------
 * 1. Write a class that satisfies gnc::AttitudeFilter (see
 *    attitude_filter_concept.hpp) or adapt it via a specialisation here.
 * 2. Add a FilterTraits<NewFilter> specialisation below.
 * 3. Add the filter's Config construction to makeConfig() in that
 *    specialisation.
 * 4. In flight_controller.h, add a new #elif branch mapping
 *    GNC_FILTER_<NAME> to the concrete type alias.
 *
 * That is the complete set of changes required — FlightController itself
 * does not need to be modified.
 *
 * Canonical tuning parameters
 * ---------------------------
 * FlightController owns a single FlightController::FilterTuning struct that
 * holds every noise parameter used across all filters. FilterTraits::makeConfig
 * picks out the relevant subset for each filter type. Parameters that have no
 * analogue in a given filter are simply ignored.
 *
 *   qGyro      [rad²/s]    Gyroscope rate noise PSD
 *   qBias      [rad²/s³]   Gyro bias random-walk PSD
 *   rAccel     [(m/s²)²]   Accelerometer noise variance
 *   rMag       [–]         Magnetometer noise variance (normalised inputs)
 *   accelGate  [m/s²]      Accel magnitude gate (MEKF only; ignored for UKF)
 *   alpha      [–]         UKF sigma-point spread (UKF only; ignored for MEKF)
 *   beta       [–]         UKF distribution prior (UKF only; ignored for MEKF)
 *   kappa      [–]         UKF secondary scaling (UKF only; ignored for MEKF)
 */

#ifndef FILTER_TRAITS_HPP
#define FILTER_TRAITS_HPP

#include "attitude_mekf.hpp"
#include "attitude_ukf.hpp"

#include <cstdint>

namespace gnc {

// ============================================================================
// Canonical tuning parameter bundle
// ============================================================================

/**
 * @brief All noise and tuning parameters across all supported filters.
 *
 * FlightController holds exactly one instance of this. FilterTraits::makeConfig
 * extracts the subset relevant to each filter.
 */
struct FilterTuning {
    // --- shared ---
    float qGyro     = 0.1f;    ///< Gyroscope rate noise PSD [rad²/s]
    float qBias     = 0.001f;  ///< Gyro bias random-walk PSD [rad²/s³]
    float rAccel    = 0.5f;    ///< Accelerometer noise variance [(m/s²)²]
    float rMag      = 1.0f;    ///< Magnetometer noise variance (normalised)

    // --- MEKF-specific ---
    float accelGate = 2.0f;    ///< Accel magnitude gate half-width [m/s²]
                               ///<   Set 0 to disable. Ignored by UKF.

    // --- UKF-specific ---
    float alpha     = 1.0f;    ///< Sigma-point spread factor. Ignored by MEKF.
    float beta      = 2.0f;    ///< Distribution prior. Ignored by MEKF.
    float kappa     = 0.0f;    ///< Secondary scaling. Ignored by MEKF.
};


// ============================================================================
// Primary template — left undefined; only specialisations are valid
// ============================================================================

/**
 * @brief Traits class for a filter policy type T.
 *
 * Each specialisation must provide:
 *   static T::Config makeConfig(const FilterTuning&)
 *   static Vector3   getBias(const T&)
 *   static uint32_t  getCovarianceFaultCount(const T&)
 */
template <typename T>
struct FilterTraits;


// ============================================================================
// Specialisation: AttitudeMekf
// ============================================================================

template <>
struct FilterTraits<AttitudeMekf> {
    using FilterType = AttitudeMekf;
    using ConfigType = AttitudeMekf::Config;
    using Vector3    = AttitudeMekf::Vector3;

    /**
     * @brief Build an AttitudeMekf::Config from the canonical tuning bundle.
     *
     * UKF-specific fields (alpha, beta, kappa) are silently ignored.
     */
    static ConfigType makeConfig(const FilterTuning& t)
    {
        return ConfigType {
            .qGyro     = t.qGyro,
            .qBias     = t.qBias,
            .rAccel    = t.rAccel,
            .rMag      = t.rMag,
            .accelGate = t.accelGate,
        };
    }

    /**
     * @brief Return the current gyro bias estimate.
     *
     * AttitudeMekf exposes getBias() directly as part of its public API.
     */
    static Vector3 getBias(const FilterType& f)
    {
        return f.getBias();
    }

    /**
     * @brief Return the hard covariance reset count.
     *
     * AttitudeMekf tracks this internally and exposes getCovarianceFaultCount().
     */
    static uint32_t getCovarianceFaultCount(const FilterType& f)
    {
        return f.getCovarianceFaultCount();
    }
};


// ============================================================================
// Specialisation: AttitudeUkf
// ============================================================================

template <>
struct FilterTraits<AttitudeUkf> {
    using FilterType = AttitudeUkf;
    using ConfigType = AttitudeUkf::Config;
    using Vector3    = AttitudeUkf::Vector3;

    /**
     * @brief Build an AttitudeUkf::Config from the canonical tuning bundle.
     *
     * MEKF-specific fields (accelGate) are silently ignored.
     */
    static ConfigType makeConfig(const FilterTuning& t)
    {
        return ConfigType {
            .alpha  = t.alpha,
            .beta   = t.beta,
            .kappa  = t.kappa,
            .qGyro  = t.qGyro,
            .qBias  = t.qBias,
            .rAccel = t.rAccel,
            .rMag   = t.rMag,
        };
    }

    /**
     * @brief Return the current gyro bias estimate.
     *
     * AttitudeUkf exposes getState() which returns the full 6D error+bias
     * vector. The bias lives in indices [3, 4, 5]. We extract it here so
     * FlightController sees a uniform getBias() surface regardless of filter.
     */
    static Vector3 getBias(const FilterType& f)
    {
        return f.getState().template segment<3>(3);
    }

    /**
     * @brief Return the hard covariance reset count.
     *
     * AttitudeUkf tracks this via a file-static counter inside
     * generateSigmaPoints() rather than a member variable. It does not
     * expose a public accessor. We return 0 here as a safe default.
     *
     * To make this observable, either:
     *   (a) Promote the counter to a member variable in AttitudeUkf and add
     *       uint32_t getCovarianceFaultCount() const — the preferred fix, or
     *   (b) Accept that UKF fault count telemetry is unavailable via this path
     *       and monitor it through the existing static counter mechanism.
     *
     * This shim ensures FlightController compiles and runs correctly either way.
     */
    static uint32_t getCovarianceFaultCount(const FilterType& /*f*/)
    {
        return 0u; // See comment above — UKF does not expose this yet.
    }
};


} // namespace gnc

#endif // FILTER_TRAITS_HPP
