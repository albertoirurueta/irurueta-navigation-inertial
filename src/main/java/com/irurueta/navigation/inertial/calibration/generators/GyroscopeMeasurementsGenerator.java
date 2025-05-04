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
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.GyroscopeNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAngularSpeedTriadNoiseEstimator;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Generates measurements for the calibration of gyroscopes by alternating
 * static and dynamic intervals where device is kept static or moved.
 * Generated measurements must be used with easy gyroscope calibrators.
 * Notice that accuracy of the gyroscope calibration is very sensitive to the
 * accuracy of detected dynamic intervals respect the average specific forces
 * during static intervals.
 * In order to increase the accuracy, calibration should be repeated trying different
 * threshold factors {@link #getThresholdFactor()}.
 * Such calibrators are the following ones:
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasEasyGyroscopeCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustEasyGyroscopeCalibrator} and all its
 * implementations.
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownBiasEasyGyroscopeCalibrator} and all its
 * implementations.
 */
public class GyroscopeMeasurementsGenerator extends
        MeasurementsGenerator<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>,
                GyroscopeMeasurementsGenerator, GyroscopeMeasurementsGeneratorListener, TimedBodyKinematics>
        implements GyroscopeNoiseRootPsdSource {

    /**
     * An angular speed triad.
     * This is reused for memory efficiency.
     */
    protected final AngularSpeedTriad angularSpeedTriad = new AngularSpeedTriad();

    /**
     * Items to be added to a generated sequence when next static period occurs.
     */
    private List<StandardDeviationTimedBodyKinematics> currentSequenceItems;

    /**
     * Accumulated noise estimator for angular speed measurements.
     */
    private final AccumulatedAngularSpeedTriadNoiseEstimator accumulatedEstimator =
            new AccumulatedAngularSpeedTriadNoiseEstimator();

    /**
     * Estimated acceleration standard deviation during initialization expressed
     * in meters per squared second (m/s^2).
     */
    private double accelerationStandardDeviation;

    /**
     * Estimated angular speed standard deviation during initialization expressed
     * in radians per second (rad/s).
     */
    private double angularSpeedStandardDeviation;

    /**
     * Estimated norm of gyroscope noise root PSD (Power Spectral Density)
     * expressed as (rad * s^-0.5).
     */
    private double angularSpeedNoiseRootPsd;

    /**
     * Previous average x-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double previousAvgX;

    /**
     * Previous average y-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double previousAvgY;

    /**
     * Previous average z-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double previousAvgZ;

    /**
     * Current average x-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double currentAvgX;

    /**
     * Current average y-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double currentAvgY;

    /**
     * Current average z-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double currentAvgZ;

    /**
     * Contains previous status while processing samples.
     */
    private TriadStaticIntervalDetector.Status previousStatus;

    /**
     * Constructor.
     */
    public GyroscopeMeasurementsGenerator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this generator.
     */
    public GyroscopeMeasurementsGenerator(final GyroscopeMeasurementsGeneratorListener listener) {
        super(listener);
    }

    /**
     * Sets time interval between input samples expressed in seconds (s).
     *
     * @param timeInterval time interval between input samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if generator is currently running.
     */
    @Override
    public void setTimeInterval(final double timeInterval) throws LockedException {
        super.setTimeInterval(timeInterval);
        accumulatedEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Resets this generator.
     *
     * @throws LockedException if generator is busy.
     */
    @Override
    public void reset() throws LockedException {
        super.reset();

        currentSequenceItems = null;

        accelerationStandardDeviation = 0.0;
        angularSpeedStandardDeviation = 0.0;

        previousAvgX = null;
        previousAvgY = null;
        previousAvgZ = null;

        currentAvgX = null;
        currentAvgY = null;
        currentAvgZ = null;

        accumulatedEstimator.reset();

        previousStatus = null;
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @return estimated average angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAvgAngularSpeedTriad() {
        return accumulatedEstimator.getAvgTriad();
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAvgAngularSpeedTriad(final AngularSpeedTriad result) {
        accumulatedEstimator.getAvgTriad(result);
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @return estimated standard deviation of angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAngularSpeedTriadStandardDeviation() {
        return accumulatedEstimator.getStandardDeviationTriad();
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAngularSpeedTriadStandardDeviation(final AngularSpeedTriad result) {
        accumulatedEstimator.getStandardDeviationTriad(result);
    }

    /**
     * Gets gyroscope base noise level that has been detected during
     * initialization expressed in radians per second (rad/s).
     * This is equal to the standard deviation of the gyroscope measurements
     * during initialization phase.
     *
     * @return gyroscope base noise level.
     */
    public double getGyroscopeBaseNoiseLevel() {
        return angularSpeedStandardDeviation;
    }

    /**
     * Gets gyroscope base noise level that has been detected during
     * initialization.
     * This is equal to the standard deviation of the gyroscope measurements
     * during initialization phase.
     *
     * @return gyroscope base noise level.
     */
    public AngularSpeed getGyroscopeBaseNoiseLevelAsMeasurement() {
        return new AngularSpeed(angularSpeedStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets gyroscope base noise level that has been detected during
     * initialization.
     * This is equal to the standard deviation of the gyroscope measurements
     * during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getGyroscopeBaseNoiseLevelAsMeasurement(final AngularSpeed result) {
        result.setValue(angularSpeedStandardDeviation);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets gyroscope base noise level PSD (Power Spectral Density)
     * expressed in (rad^2/s).
     *
     * @return gyroscope base noise level PSD.
     */
    public double getGyroscopeBaseNoiseLevelPsd() {
        return angularSpeedNoiseRootPsd * angularSpeedNoiseRootPsd;
    }

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    @Override
    public double getGyroscopeBaseNoiseLevelRootPsd() {
        return angularSpeedNoiseRootPsd;
    }

    /**
     * Post process provided input sample.
     *
     * @param sample an input sample.
     * @throws LockedException if generator is busy.
     */
    @Override
    protected void postProcess(final TimedBodyKinematics sample) throws LockedException {
        final var status = staticIntervalDetector.getStatus();

        if (status == TriadStaticIntervalDetector.Status.INITIALIZING) {
            sample.getKinematics().getAngularRateTriad(angularSpeedTriad);
            accumulatedEstimator.addTriad(angularSpeedTriad);
        }

        // while we are in a dynamic interval, we must record all timed kinematics
        // along with accelerometer and gyroscope standard deviations
        if (status == TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL) {
            if (isDynamicIntervalSkipped()) {
                // dynamic interval has been skipped because there were too many
                // items in the sequence.
                currentSequenceItems = null;
            } else {
                if (previousStatus == TriadStaticIntervalDetector.Status.STATIC_INTERVAL) {
                    previousAvgX = staticIntervalDetector.getAccumulatedAvgX();
                    previousAvgY = staticIntervalDetector.getAccumulatedAvgY();
                    previousAvgZ = staticIntervalDetector.getAccumulatedAvgZ();
                }

                addSequenceItem(sample);
            }
        } else if (status == TriadStaticIntervalDetector.Status.STATIC_INTERVAL
                && previousStatus == TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL
                && currentSequenceItems != null && !currentSequenceItems.isEmpty()) {

            currentAvgX = staticIntervalDetector.getInstantaneousAvgX();
            currentAvgY = staticIntervalDetector.getInstantaneousAvgY();
            currentAvgZ = staticIntervalDetector.getInstantaneousAvgZ();

            // we have all required data to generate a sequence
            BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = null;
            if (listener != null) {
                sequence = new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(previousAvgX, previousAvgY, previousAvgZ);
                sequence.setItems(currentSequenceItems);
                sequence.setAfterMeanSpecificForceCoordinates(currentAvgX, currentAvgY, currentAvgZ);
            }

            currentSequenceItems = null;

            if (listener != null) {
                listener.onGeneratedMeasurement(this, sequence);
            }
        }
    }

    /**
     * Gets corresponding acceleration triad from provided input sample.
     * This method must store the result into {@link #triad}.
     *
     * @param sample input sample.
     */
    @Override
    protected void getAccelerationTriadFromInputSample(final TimedBodyKinematics sample) {
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
        previousStatus = TriadStaticIntervalDetector.Status.STATIC_INTERVAL;
    }

    /**
     * Handles a dynamic-to-static interval change.
     */
    @Override
    protected void handleDynamicToStaticChange() {
        previousStatus = TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL;
    }

    /**
     * Handles an initialization completion.
     */
    @Override
    protected void handleInitializationCompleted() {
        accelerationStandardDeviation = staticIntervalDetector.getBaseNoiseLevel();
        angularSpeedStandardDeviation = accumulatedEstimator.getStandardDeviationNorm();
        angularSpeedNoiseRootPsd = accumulatedEstimator.getNoiseRootPsdNorm();

        previousStatus = staticIntervalDetector.getStatus();
    }

    /**
     * Handles an error during initialization.
     */
    @Override
    protected void handleInitializationFailed() {
        previousStatus = null;

        try {
            accumulatedEstimator.reset();
        } catch (final LockedException ignore) {
            // no action needed
        }
    }

    /**
     * Adds an item to current sequence items.
     *
     * @param sample sample to generate a sequence item from.
     */
    private void addSequenceItem(final TimedBodyKinematics sample) {
        if (currentSequenceItems == null) {
            currentSequenceItems = new ArrayList<>();
        }

        final var kinematics = new BodyKinematics(sample.getKinematics());
        final var timestampSeconds = sample.getTimestampSeconds();
        final var stdTimedKinematics = new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds,
                accelerationStandardDeviation, angularSpeedStandardDeviation);
        currentSequenceItems.add(stdTimedKinematics);
    }
}
