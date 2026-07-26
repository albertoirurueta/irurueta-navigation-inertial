# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/albertoirurueta/irurueta-navigation-inertial/compare/1.9.0...HEAD
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
