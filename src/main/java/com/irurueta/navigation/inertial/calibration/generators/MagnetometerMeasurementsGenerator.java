/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityTriadNoiseEstimator;

/**
 * Generates measurements for the calibration of magnetometers by alternating
 * static and dynamic intervals where device is kept static or moved.
 * Generated measurements must be used with magnetometer calibrators based
 * on the knowledge of position on Earth and time instant.
 * Such calibrators are the following ones:
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronPositionAndInstantMagnetometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownPositionAndInstantMagnetometerCalibrator}
 * and all its implementations.
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownHardIronPositionAndInstantMagnetometerCalibrator}
 * and all its implementations.
 */
public class MagnetometerMeasurementsGenerator extends
        MeasurementsGenerator<StandardDeviationBodyMagneticFluxDensity,
                MagnetometerMeasurementsGenerator, MagnetometerMeasurementsGeneratorListener,
                BodyKinematicsAndMagneticFluxDensity> {

    /**
     * Accumulated noise estimator for magnetic flux density measurements.
     */
    private final AccumulatedMagneticFluxDensityTriadNoiseEstimator accumulatedEstimator =
            new AccumulatedMagneticFluxDensityTriadNoiseEstimator();

    /**
     * Accumulated average x-coordinate of magnetic flux density while body remains in a static interval and
     * expressed in Teslas (T).
     */
    private double avgBx;

    /**
     * Accumulated average y-coordinate of magnetic flux density while body remains in a static interval and
     * expressed in Teslas (T).
     */
    private double avgBy;

    /**
     * Accumulated average z-coordinate of magnetic flux density while body remains in a static interval and
     * expressed in Teslas (T).
     */
    private double avgBz;

    /**
     * Contains standard deviation of magnetic flux density during initialization (i.e. base noise level)
     * expressed in Teslas (T).
     */
    private double stdBNorm;

    /**
     * Constructor.
     */
    public MagnetometerMeasurementsGenerator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this generator.
     */
    public MagnetometerMeasurementsGenerator(final MagnetometerMeasurementsGeneratorListener listener) {
        super(listener);
    }

    /**
     * Resets this generator.
     *
     * @throws LockedException if generator is busy.
     */
    @Override
    public void reset() throws LockedException {
        super.reset();

        accumulatedEstimator.reset();
    }

    /**
     * Post process provided input sample.
     *
     * @param sample an input sample.
     * @throws LockedException if generator is busy.
     */
    @Override
    protected void postProcess(final BodyKinematicsAndMagneticFluxDensity sample) throws LockedException {
        final var b = sample.getMagneticFluxDensity();
        if (staticIntervalDetector.getStatus() == TriadStaticIntervalDetector.Status.STATIC_INTERVAL
                || staticIntervalDetector.getStatus() == TriadStaticIntervalDetector.Status.INITIALIZING) {

            accumulatedEstimator.addTriad(b.getBx(), b.getBy(), b.getBz());
            avgBx = accumulatedEstimator.getAvgX();
            avgBy = accumulatedEstimator.getAvgY();
            avgBz = accumulatedEstimator.getAvgZ();

        } else {
            accumulatedEstimator.reset();
        }
    }

    /**
     * Gets corresponding acceleration triad from provided input sample.
     * This method must store the result into {@link #triad}.
     *
     * @param sample input sample.
     */
    @Override
    protected void getAccelerationTriadFromInputSample(final BodyKinematicsAndMagneticFluxDensity sample) {
        sample.getKinematics().getSpecificForceTriad(triad);
    }

    /**
     * Handles a static-to-dynamic interval change.
     *
     * @param accumulatedAvgX average x-coordinate of measurements during last
     *                        static period expressed in meters per squared
     *                        second (m/s^2).
     * @param accumulatedAvgY average y-coordinate of specific force during last
     *                        static period expressed in meters per squared
     *                        second (m/s^2).
     * @param accumulatedAvgZ average z-coordinate of specific force during last
     *                        static period expressed in meters per squared
     *                        second (m/s^2).
     * @param accumulatedStdX standard deviation of x-coordinate of measurements
     *                        during last static period expressed in meters per
     *                        squared second (m/s^2).
     * @param accumulatedStdY standard deviation of y-coordinate of measurements
     *                        during last static period expressed in meters per
     *                        squared second (m/s^2).
     * @param accumulatedStdZ standard deviation of z-coordinate of measurements
     *                        during last static period expressed in meters per
     *                        squared second (m/s^2).
     */
    @Override
    protected void handleStaticToDynamicChange(
            final double accumulatedAvgX, final double accumulatedAvgY, final double accumulatedAvgZ,
            final double accumulatedStdX, final double accumulatedStdY, final double accumulatedStdZ) {

        if (!isStaticIntervalSkipped()) {

            final var measurement = new StandardDeviationBodyMagneticFluxDensity(
                    new BodyMagneticFluxDensity(avgBx, avgBy, avgBz), stdBNorm);

            if (listener != null) {
                listener.onGeneratedMeasurement(this, measurement);
            }
        }
    }

    /**
     * Handles a dynamic-to-static interval change.
     */
    @Override
    protected void handleDynamicToStaticChange() {
        // no action needed.
    }

    /**
     * Handles an initialization completion.
     */
    @Override
    protected void handleInitializationCompleted() {
        stdBNorm = accumulatedEstimator.getStandardDeviationNorm();
    }

    /**
     * Handles an error during initialization.
     */
    @Override
    protected void handleInitializationFailed() {
        try {
            accumulatedEstimator.reset();
        } catch (final LockedException ignore) {
            // no action needed
        }
    }
}
