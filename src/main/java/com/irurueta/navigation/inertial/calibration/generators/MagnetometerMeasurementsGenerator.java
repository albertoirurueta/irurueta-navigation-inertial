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

import java.util.ArrayDeque;

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
 * <p>
 * Generated measurements contain the average magnetic flux density of each static interval, computed
 * only with the samples that the (centred window) detector classified as static, and its standard deviation
 * is the one of the average, that is, the mean standard deviation of the three coordinates divided by the
 * square root of the number of averaged samples. The initialization period is static, so its samples are
 * part of the first static interval.
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
     * Magnetic flux density samples that have been received but not yet classified by the detector,
     * which classifies the sample at the centre of its window. Each entry contains x, y, z coordinates
     * expressed in Teslas (T).
     */
    private final ArrayDeque<double[]> pendingSamples = new ArrayDeque<>();

    /**
     * Indicates that the sample that completed initialization has not been processed yet.
     */
    private boolean initializationSamplePending;

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
        pendingSamples.clear();
        initializationSamplePending = false;
    }

    /**
     * Post process provided input sample.
     * Detector classifies the sample at the centre of its window, so samples are buffered until they
     * are classified, and only samples classified as static contribute to the accumulated average.
     * The initialization period is static, so its samples are part of the first static interval.
     *
     * @param sample an input sample.
     * @throws LockedException if generator is busy.
     */
    @Override
    protected void postProcess(final BodyKinematicsAndMagneticFluxDensity sample) throws LockedException {
        final var b = sample.getMagneticFluxDensity();
        final var status = staticIntervalDetector.getStatus();

        if (status == TriadStaticIntervalDetector.Status.INITIALIZING) {
            accumulatedEstimator.addTriad(b.getBx(), b.getBy(), b.getBz());
            return;
        }

        if (status == TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED && initializationSamplePending) {
            // sample that completed initialization belongs to the initialization period
            initializationSamplePending = false;
            accumulatedEstimator.addTriad(b.getBx(), b.getBy(), b.getBz());
            return;
        }

        if (status != TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED
                && status != TriadStaticIntervalDetector.Status.STATIC_INTERVAL
                && status != TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL) {
            return;
        }

        pendingSamples.addLast(new double[]{b.getBx(), b.getBy(), b.getBz()});

        if (status == TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED) {
            // first classified sample has not been reached yet
            return;
        }

        // oldest pending sample is the one that has just been classified
        final var classified = pendingSamples.pollFirst();

        if (status == TriadStaticIntervalDetector.Status.STATIC_INTERVAL) {
            accumulatedEstimator.addTriad(classified[0], classified[1], classified[2]);
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

        // accumulated estimator only contains samples classified as static
        final var n = accumulatedEstimator.getNumberOfProcessedSamples();
        if (!isStaticIntervalSkipped() && n > 0) {

            // the standard deviation of the average of N static samples is sigma / sqrt(N), where
            // sigma is the mean standard deviation of the three coordinates
            final var avgStd = (accumulatedEstimator.getStandardDeviationX()
                    + accumulatedEstimator.getStandardDeviationY()
                    + accumulatedEstimator.getStandardDeviationZ()) / 3.0;

            final var measurement = new StandardDeviationBodyMagneticFluxDensity(
                    new BodyMagneticFluxDensity(accumulatedEstimator.getAvgX(), accumulatedEstimator.getAvgY(),
                            accumulatedEstimator.getAvgZ()), avgStd / Math.sqrt(n));

            if (listener != null) {
                listener.onGeneratedMeasurement(this, measurement);
            }
        }

        try {
            accumulatedEstimator.reset();
        } catch (final LockedException ignore) {
            // never happens
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
        pendingSamples.clear();
        initializationSamplePending = true;
    }

    /**
     * Handles an error during initialization.
     */
    @Override
    protected void handleInitializationFailed() {
        pendingSamples.clear();
        initializationSamplePending = false;
        try {
            accumulatedEstimator.reset();
        } catch (final LockedException ignore) {
            // no action needed
        }
    }
}
