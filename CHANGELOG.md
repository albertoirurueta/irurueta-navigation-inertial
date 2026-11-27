# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Checkstyle now uses the repository's own `checkstyle.xml` when run from the command line (`mvn checkstyle:checkstyle`),
  not only in the Maven site report, so local runs and the published report measure the same rules (issue #47).
- The Checkstyle policy in `checkstyle.xml` is relaxed for patterns that are intentional in this numerical library:
  `HiddenField` ignores constructor and setter parameters (including setters that return their class), `MagicNumber`
  ignores `-1, 0, 1, 2, 3, 0.5`, `hashCode()` methods, field initializers and annotation values, and
  `ParameterNumber` allows up to 15 parameters and ignores overridden methods. `@SuppressWarnings("checkstyle:...")`
  annotations are now honoured (issue #47).
- Checkstyle findings under `src/main/java` are reduced from 20,088 to 4,370 through policy changes and formatting,
  import, modifier, declaration order, Javadoc and line length fixes, with no public API changes. The remaining
  findings (`MagicNumber`, `HiddenField`, `ParameterNumber`, `ThrowsCount`) are numerical constants or public API
  signatures (issue #47).
- SpotBugs now uses the repository's own `spotbugs-exclude.xml` filter, both from the command line
  (`mvn spotbugs:spotbugs`) and in the Maven site report. It excludes only `CT_CONSTRUCTOR_THROW` (constructors
  validate arguments through setters by design) and `EI_EXPOSE_REP`/`EI_EXPOSE_REP2` (numerical APIs share
  `Matrix`, array and collection instances on purpose), scoped to this library's packages; SpotBugs reports no
  findings under `src/main/java` (issue #49).

### Fixed

- `WMMLoader` now reads World Magnetic Model coefficient files as UTF-8 instead of depending on the platform default
  charset (issue #49).
- `clone()` of state, configuration and measurement classes no longer calls the overridable `copyTo` method, and
  clones no longer share mutable objects (matrices, arrays, nested values) with the original (issue #49).
- `StandardDeviationBodyMagneticFluxDensity.clone()` previously returned a shallow copy sharing the magnetic flux
  density with the original; it now returns a deep copy (issue #49).

## [1.12.0] - 2026-11-25

### Fixed

- Windowed noise estimators (`WindowedTriadNoiseEstimator`, `WindowedMeasurementNoiseEstimator` and subclasses) now
  compute the average and the unbiased variance over the number of samples actually stored in the window, instead of
  dividing by the full window size while it was not yet full (issue #41, N8).
- `TriadStaticIntervalDetector` now classifies the sample at the centre of the window (as imu_tk), so interval
  boundaries are symmetric and static averages no longer contain motion samples. Status changes are delayed by half a
  window, available as `getDetectionDelaySamples()` (issue #41, N3).
- `GyroscopeMeasurementsGenerator` and the combined generators now only generate a sequence when both neighbouring
  static intervals are valid, join the dynamic intervals around a too short static interval, and emit the sequence when
  the following static interval ends, using the average of that whole interval as the "after" gravity vector
  (issue #41, N4). When no more samples are going to be processed (e.g. a recording that ends at rest), the new
  `flush()` method generates the sequence that is still pending.
- `MagnetometerMeasurementsGenerator` is now aligned with the centred detection window: only samples classified as
  static are averaged (previously, with the centred window, about half a window of motion samples was included), the
  initialization period is part of the first static interval, and the reported standard deviation is the one of the
  average (mean per-axis standard deviation of the interval divided by `sqrt(N)`) instead of the norm of the standard
  deviation of the initialization period (issue #41, N3 and N6).
- The initialization period is now the first static interval of the generators (its average is kept, instead of being
  reset), so a motion right after initialization no longer gets a zero "before" gravity vector (issue #41, N9).
- Standard deviations are now consistent with the quantity they describe (issue #41, N6): averaged static
  measurements report `sigma/sqrt(N)`; the gravity-norm accelerometer and norm-based magnetometer calibrators give the
  fitter `2*g*sigma` (resp. `2*|B|*sigma`) for the squared-norm residual, so chi-square and covariance are properly
  scaled; gyroscope sequences carry a per-component gravity-direction
  standard deviation.
- The easy gyroscope calibrators (`EasyGyroscopeCalibrator`, `KnownBiasEasyGyroscopeCalibrator`) now fit a
  three-component residual (`R^T * u_before` against `u_after`) with `LevenbergMarquardtMultiVariateFitter` and a
  numerical Jacobian that does not vanish near zero, instead of the scalar distance between unit vectors (issue #41,
  N5). The robust variants already scored the norm of that residual.

### Added

- `BodyKinematicsSequence.getGravityDirectionStandardDeviation()` / `setGravityDirectionStandardDeviation(...)`
  (dimensionless, default 0), populated by the gyroscope generators and used by the easy gyroscope calibrators.
- `TriadStaticIntervalDetector.getDetectionDelaySamples()` and `MeasurementsGenerator.getDetectionDelaySamples()`,
  which report the half-window delay of the detected status changes.

### Changed

- **Behaviour change, affects `irurueta-navigation-inertial-extra`
  ([albertoirurueta/irurueta-navigation-inertial-extra#29](https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/issues/29))**:
  `DEFAULT_USE_COMMON_Z_AXIS` is now `true` in the gravity-norm accelerometer calibrators (`KnownGravityNorm`,
  `KnownBiasAndGravityNorm`, `KnownPosition`, `KnownBiasAndPosition` and their `RobustKnown[BiasAnd]GravityNorm` and
  `RobustKnown[BiasAnd]Position` variants) and
  the norm-based magnetometer calibrators (`KnownMagneticFluxDensityNorm`, `KnownHardIronMagneticFluxDensityNorm`,
  `KnownPositionAndInstant`, `KnownHardIronPositionAndInstant` and robust variants). The general (full 3x3) mode
  remains available with `setCommonAxisUsed(false)`, but its matrix is only defined up to a rotation. Minimum
  measurements with the new default are 10 / 7 (unknown / known bias or hard iron) instead of 13 / 10 (issue #41, N1).
- **Behaviour change**: `EasyGyroscopeCalibrator`, `KnownBiasEasyGyroscopeCalibrator`, `RobustEasyGyroscopeCalibrator`
  and `RobustKnownBiasEasyGyroscopeCalibrator` now default to `commonAxisUsed = false` and
  `estimateGDependentCrossBiases = false`, as in the paper. Minimum sequences with the new defaults are 13 and 10.
  The Turntable calibrators are unchanged (issue #41, N2).
- **Behaviour change**: `TriadStaticIntervalDetector` (and the `Acceleration`, `AngularSpeed` and `MagneticFluxDensity`
  detectors and all generators using them) now decides with the *variance-norm* statistic of imu_tk instead of the
  standard-deviation norm. The meaning of every `thresholdFactor`, `instantaneousNoiseLevelFactor` and
  `baseNoiseLevelAbsoluteThreshold` changes, with `k_reference ~ k^2`; `DEFAULT_THRESHOLD_FACTOR` goes from 2.0 to 4.0 and
  `DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR` from 2.0 to 9.0. Previously tuned values (for example in
  `irurueta-navigation-inertial-extra` optimizers) must be revisited (issue #41, N7).
- The measurement-typed accessors of the detector and the generators (`getBaseNoiseLevelAsMeasurement`,
  `getThresholdAsMeasurement`, `getBaseNoiseLevelAbsoluteThresholdAsMeasurement`,
  `getAccelerometerBaseNoiseLevelAsMeasurement` and the measurement-typed `setBaseNoiseLevelAbsoluteThreshold`) keep
  working in the unit of the sensor: getters return the square root of the variance-based value and the setter stores
  the square of the provided measurement, so unit conversions remain valid (issue #41, N7).
- The initialization samples now count towards `minStaticSamples` for the first static interval, for every
  generator, so a motion starting shortly after initialization no longer raises `onStaticIntervalSkipped` nor drops
  the first accelerometer or magnetometer measurement (issue #41, N9).
- A gyroscope sequence obtained by joining dynamic intervals is discarded, raising `onDynamicIntervalSkipped`, when
  it exceeds `maxDynamicSamples` (issue #41, N4).
- Detector status changes are delayed by half a window (centred window), so listener callbacks and generator outputs
  occur `getDetectionDelaySamples()` samples after the sample they refer to (issue #41, N3).
- `AccelerometerMeasurementsGenerator` standard deviations are divided by `sqrt(N)`, and the standard deviation given
  to the fitter by the norm-based calibrators is `2*g*sigma`, so reported chi-square, covariance and weights change
  (issue #41, N6).
- The easy gyroscope residual is now three-component and weighted by the sequence gravity-direction standard deviation
  (issue #41, N5).
- Updated internal dependencies: `irurueta-numerical` to 1.7.1, `irurueta-geometry` to 1.6.2 and
  `irurueta-navigation` to 1.8.3. The easy gyroscope calibrators use the `JacobianEstimator` of `irurueta-numerical`
  with a minimum step (issue #43).

## [1.11.0] - 2026-11-22

### Fixed

- `INSLooselyCoupledKalmanFilteredEstimator`, `INSGNSSLooselyCoupledKalmanFilteredEstimator` and
  `INSGNSSTightlyCoupledKalmanFilteredEstimator` now integrate every provided IMU sample with the ECEF inertial
  navigation equations (previously only samples one epoch interval apart were used), and each Kalman filter epoch
  uses the resulting INS-propagated solution as its predicted state, applying the closed-loop correction back to
  it, as in Groves' `Loosely_coupled_INS_GNSS.m` / `Tightly_coupled_INS_GNSS.m` reference algorithms.
- `INSGNSSLooselyCoupledKalmanFilteredEstimator` now uses the GNSS least-squares solution as the Kalman
  measurement at its own timestamp instead of navigating it forward and feeding the INS solution back as a
  measurement.
- `INSGNSSTightlyCoupledKalmanFilteredEstimator` now re-orthonormalises the attitude after each Kalman epoch and
  initialises the state's receiver clock offset and drift from the least-squares solution.
- `ECEFKinematicsEstimator` now computes the Earth-rotation term of the average body-to-ECEF coordinate
  transformation matrix (Groves eq. 5.85) from the attitude at the start of the interval, matching
  `ECEFInertialNavigator`.
- The `(frame, oldFrame)` overloads of `ECEFKinematicsEstimator` and `ECIKinematicsEstimator` now evaluate
  gravity/gravitation at the start-of-interval position (the one in `oldFrame`) instead of the end-of-interval one,
  so that they are the exact inverse of `ECEFInertialNavigator` / `ECIInertialNavigator`.
- The epoch gate of the three filtered estimators now accepts fixes/measurements arriving exactly one epoch
  interval after the previous Kalman epoch (they were previously rejected), also when the difference of both
  timestamps is rounded below the epoch interval (e.g. `0.7 - 0.2 < 0.5`).
- The first Kalman epoch of the three filtered estimators now propagates the covariance over the time elapsed since
  the first IMU sample, instead of using a zero propagation interval.
- `INSGNSSTightlyCoupledKalmanFilteredEstimator` now integrates IMU samples provided before the first GNSS
  measurements, so that attitude changes before position and velocity are initialised are not lost.

### Added

- `INSLooselyCoupledKalmanFilteredEstimator.updatePositionAndVelocity(...)` (three overloads), which provides a
  position and velocity fix and runs a Kalman filter epoch with it.
- `getLastKinematicsTimestamp()` and `getLastKinematicsTimestampAsTime(...)` on
  `INSLooselyCoupledKalmanFilteredEstimator` and `INSGNSSTightlyCoupledKalmanFilteredEstimator`, returning the
  timestamp of the last integrated IMU sample.

### Changed

- Behaviour of the filtered estimators' methods changed, with unchanged signatures:
  - `INSLooselyCoupledKalmanFilteredEstimator.update` and `updateBodyKinematics` of both INS/GNSS estimators
    integrate every sample whose timestamp is greater than the previous one (the first sample only sets the time
    reference), regardless of the epoch interval, and no longer run a Kalman filter epoch. Their return value now
    tells whether the sample was integrated.
  - `updateMeasurements` of both INS/GNSS estimators runs the Kalman filter epoch itself, at the timestamp of the
    measurements, whenever the estimator is ready to propagate.
  - `INSLooselyCoupledKalmanFilteredEstimator.propagate` is deprecated: it only initialises/synchronises the Kalman
    state with the INS solution and always returns `false`; use `updatePositionAndVelocity` instead.
    `INSGNSSLooselyCoupledKalmanFilteredEstimator.propagate` likewise always returns `false`, and
    `INSGNSSTightlyCoupledKalmanFilteredEstimator.propagate` only runs an epoch for measurements not yet consumed
    by one.
  - `INSLooselyCoupledKalmanFilteredEstimator.setFrame` also synchronises an already initialised Kalman state with
    the provided frame, preserving estimated biases and covariance.
  - `INSGNSSTightlyCoupledKalmanFilteredEstimator.updateBodyKinematics` reports inertial navigation failures as
    `INSGNSSException`.

## [1.10.2] - 2026-11-16

### Changed

- Updated internal dependencies: `irurueta-numerical` to 1.6.1, `irurueta-geometry` to 1.6.1, `irurueta-units` to
  1.4.1, `irurueta-algebra` to 1.4.1, and `irurueta-navigation` to 1.8.2. No changes to this library's own source.

## [1.10.1] - 2026-07-28

### Fixed

- Fixed flaky test assertions in magnetometer calibrator tests; no functional changes to library source.

## [1.10.0] - 2026-07-27

### Changed

- Build now targets Java 21 (up from Java 17).

## [1.9.0] - 2026-03-05

### Changed

- Updated internal dependencies: `irurueta-numerical` to 1.5.0, `irurueta-geometry` to 1.5.0, and
  `irurueta-navigation` to 1.7.1. No changes to this library's own source.

## [1.8.0] - 2026-02-03

### Added

- New `DistanceTriad` and `SpeedTriad` classes, representing triads of distance/position and speed measurements,
  mirroring the existing acceleration/angular-speed/magnetic-flux-density triads.
- New abstract `copy()` method on `Triad`, implemented by all concrete triad subclasses.

### Changed

- **Breaking:** `Triad<U, T>` is now `Triad<U, M, T>` — a third, self-referencing type parameter for the concrete
  triad type was added. Code extending or directly parameterizing `Triad` must add the new type parameter.

## [1.7.1] - 2025-12-26

### Fixed

- Fixed a flaky test assertion; no functional changes to library source.

## [1.7.0] - 2025-12-26

### Added

- New calibration quality-of-fit APIs on all non-linear/robust accelerometer, gyroscope, and magnetometer
  calibrators: `getEstimatedChiSqDegreesOfFreedom()`, `getEstimatedReducedChiSq()`, `getEstimatedP()`, and
  `getEstimatedQ()`, exposing statistical quality/confidence of the estimated calibration fit.

### Changed

- Bumped `irurueta-numerical` and `irurueta-geometry` to 1.4.0, and `irurueta-navigation` to 1.6.0.

## [1.6.4] - 2025-09-23

### Changed

- Updated dependency versions in `pom.xml`; no functional changes to library source.

## [1.6.3] - 2025-09-20

### Changed

- Updated build plugins and CI workflows; no functional changes to library source.

## [1.6.2] - 2025-05-08

Test-only fixes; no functional changes to library source.

## [1.6.1] - 2025-05-07

Test-only fixes; no functional changes to library source.

## [1.6.0] - 2025-05-07

### Changed

- Updated the World Magnetic Model (WMM) coefficients from the 2020-2025 epoch to the 2025-2030 epoch,
  affecting magnetic declination/inclination/intensity values returned by `WMMEarthMagneticFluxDensityEstimator`.
- Internal refactor of field naming conventions and JUnit 5 migration across the codebase; no public API or
  behavior change.

## [1.5.1] - 2024-09-29

Test/tooling-only release; no functional changes to library source.

## [1.5.0] - 2024-09-29

### Added

- Three new gyroscope quaternion step integrators: `SuhQuaternionStepIntegrator`, `TrawnyQuaternionStepIntegrator`,
  and `YuanQuaternionStepIntegrator`, with corresponding new `QuaternionStepIntegratorType` enum values (`SUH`,
  `TRAWNY`, `YUAN`).

### Changed

- Build now targets Java 17.
- Dependency bumps: `irurueta-numerical` to 1.2.1, `irurueta-geometry` to 1.2.0, `irurueta-units` to 1.2.0,
  `irurueta-algebra` to 1.2.0, `irurueta-navigation` to 1.4.1.

## [1.4.0] - 2022-10-07

### Added

- `NEDInertialNavigator`: new overloads of `navigate(...)`/`navigateNED(...)` accepting a configurable
  `accuracyThreshold` parameter, and a new `DEFAULT_ACCURACY_THRESHOLD` constant.

### Changed

- `NEDInertialNavigator`: attitude-matrix handling now validates the resulting body-to-NED rotation matrix
  against a (now configurable) accuracy threshold instead of forcibly re-normalizing it by determinant scaling.
- Bumped `irurueta-navigation` dependency to 1.3.1.

### Fixed

- `ECEFInertialNavigator`: fixed the attitude update using the wrong Earth-rotation matrix direction
  (`ecefToEciMatrixFromAngle` instead of `eciToEcefMatrixFromAngle`) — a correctness bug in the ECEF navigation
  equations.

## [1.3.0] - 2022-08-01

### Added

- `TimeIntervalEstimator` gained a copy constructor and `copyFrom`/`copyTo` methods.
- New pluggable gyroscope attitude-integration strategy: `QuaternionStepIntegrator` interface plus
  `EulerQuaternionStepIntegrator`, `MidPointQuaternionStepIntegrator`, and `RungeKuttaQuaternionStepIntegrator`
  implementations, and a `QuaternionStepIntegratorType` enum.

### Changed

- **Breaking:** `QuaternionIntegrator.integrateGyroSequence(...)` now takes an additional
  `QuaternionStepIntegratorType` parameter and declares `throws RotationException`. Internal callers (the "easy"
  gyroscope calibrators) were updated to keep using RK4, so their externally observable behavior is unchanged.
- Bumped `irurueta-navigation` dependency to 1.2.0.

## [1.2.0] - 2022-05-20

### Added

- New family of magnetometer calibrators based on a known magnetic flux density norm:
  `KnownMagneticFluxDensityNormMagnetometerCalibrator`, `KnownHardIronMagneticFluxDensityNormMagnetometerCalibrator`,
  plus their base classes, listeners, and robust variants (RANSAC, MSAC, LMedS, PROSAC, PROMedS).

### Changed

- Internal refactor of `KnownPositionAndInstantMagnetometerCalibrator`, `KnownHardIronPositionAndInstantMagnetometerCalibrator`,
  and their robust variants to share code with the new base calibrator classes.

### Removed

- **Breaking:** `getMagneticModel()`/`setMagneticModel(WorldMagneticModel)` removed from the
  `MagnetometerCalibrator` interface (concrete calibrator classes that use a magnetic model still expose their
  own accessors).

## [1.1.1c] - 2022-01-27

Test-only fix; no functional changes to library source.

## [1.1.1b] - 2022-01-27

Test-only fix; no functional changes to library source.

## [1.1.1] - 2022-01-27

### Fixed

- `RobustKnownBiasAndGravityNormAccelerometerCalibrator` now correctly derives its
  `MINIMUM_MEASUREMENTS_COMMON_Z_AXIS`/`MINIMUM_MEASUREMENTS_GENERAL` constants from
  `KnownBiasAndPositionAccelerometerCalibrator` instead of the unrelated `KnownPositionAccelerometerCalibrator`,
  so robust accelerometer calibrators now request the proper minimum number of measurements.

## [1.1.0] - 2021-12-12

### Changed

- Bumped compile-scope dependencies (`irurueta-numerical`, `irurueta-geometry`, `irurueta-units`,
  `irurueta-algebra`, `irurueta-navigation`) from 1.0.0 to 1.1.0.

## [1.0.0] - 2021-12-10

### Added

- Initial release of the inertial GNSS/INS navigation library, providing:
  - Core inertial data types (`BodyKinematics`, `BodyMagneticFluxDensity`, `ECEFGravity`, `ECIGravitation`,
    `NEDGravity`, `RadiiOfCurvature`, etc.).
  - Loosely/tightly coupled INS/GNSS Kalman filter estimators and related config/state classes.
  - Kinematics, gravity/gravitation, leveling, position/velocity, and attitude estimators (ECEF/ECI/NED variants).
  - ECEF/ECI/NED inertial navigators.
  - A full sensor calibration framework, including accelerometer, gyroscope, and magnetometer calibrators (known
    bias/position/frame variants, and robust — RANSAC/MSAC/PROSAC/etc. — estimators), bias/noise estimators,
    static/dynamic interval detectors, and kinematics/magnetic-flux-density sequence generators.
  - World Magnetic Model (WMM) support for estimating Earth's magnetic flux density.

### Fixed

- Ensured correct `Serializable` implementation (`serialVersionUID`, etc.) across inertial data classes.

[Unreleased]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.12.0...HEAD
[1.12.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.11.0...1.12.0
[1.11.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.10.2...1.11.0
[1.10.2]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.10.1...1.10.2
[1.10.1]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.10.0...1.10.1
[1.10.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.9.0...1.10.0
[1.9.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.8.0...1.9.0
[1.8.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.7.1...1.8.0
[1.7.1]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.7.0...1.7.1
[1.7.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.6.4...1.7.0
[1.6.4]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.6.3...1.6.4
[1.6.3]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.6.2...1.6.3
[1.6.2]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.6.1...1.6.2
[1.6.1]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.6.0...1.6.1
[1.6.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.5.1...1.6.0
[1.5.1]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.5.0...1.5.1
[1.5.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.4.0...1.5.0
[1.4.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.3.0...1.4.0
[1.3.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.2.0...1.3.0
[1.2.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.1.1c...1.2.0
[1.1.1c]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.1.1b...1.1.1c
[1.1.1b]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.1.1...1.1.1b
[1.1.1]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.1.0...1.1.1
[1.1.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.0.0...1.1.0
[1.0.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial/releases/tag/1.0.0
