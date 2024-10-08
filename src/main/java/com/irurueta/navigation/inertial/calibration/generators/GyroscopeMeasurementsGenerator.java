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
    protected final AngularSpeedTriad mAngularSpeedTriad = new AngularSpeedTriad();

    /**
     * Items to be added to a generated sequence when next static period occurs.
     */
    private List<StandardDeviationTimedBodyKinematics> mCurrentSequenceItems;

    /**
     * Accumulated noise estimator for angular speed measurements.
     */
    private final AccumulatedAngularSpeedTriadNoiseEstimator mAccumulatedEstimator =
            new AccumulatedAngularSpeedTriadNoiseEstimator();

    /**
     * Estimated acceleration standard deviation during initialization expressed
     * in meters per squared second (m/s^2).
     */
    private double mAccelerationStandardDeviation;

    /**
     * Estimated angular speed standard deviation during initialization expressed
     * in radians per second (rad/s).
     */
    private double mAngularSpeedStandardDeviation;

    /**
     * Estimated norm of gyroscope noise root PSD (Power Spectral Density)
     * expressed as (rad * s^-0.5).
     */
    private double mAngularSpeedNoiseRootPsd;

    /**
     * Previous average x-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mPreviousAvgX;

    /**
     * Previous average y-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mPreviousAvgY;

    /**
     * Previous average z-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mPreviousAvgZ;

    /**
     * Current average x-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mCurrentAvgX;

    /**
     * Current average y-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mCurrentAvgY;

    /**
     * Current average z-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mCurrentAvgZ;

    /**
     * Contains previous status while processing samples.
     */
    private TriadStaticIntervalDetector.Status mPreviousStatus;

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
        mAccumulatedEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Resets this generator.
     *
     * @throws LockedException if generator is busy.
     */
    @Override
    public void reset() throws LockedException {
        super.reset();

        mCurrentSequenceItems = null;

        mAccelerationStandardDeviation = 0.0;
        mAngularSpeedStandardDeviation = 0.0;

        mPreviousAvgX = null;
        mPreviousAvgY = null;
        mPreviousAvgZ = null;

        mCurrentAvgX = null;
        mCurrentAvgY = null;
        mCurrentAvgZ = null;

        mAccumulatedEstimator.reset();

        mPreviousStatus = null;
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @return estimated average angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAvgAngularSpeedTriad() {
        return mAccumulatedEstimator.getAvgTriad();
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAvgAngularSpeedTriad(final AngularSpeedTriad result) {
        mAccumulatedEstimator.getAvgTriad(result);
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @return estimated standard deviation of angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAngularSpeedTriadStandardDeviation() {
        return mAccumulatedEstimator.getStandardDeviationTriad();
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAngularSpeedTriadStandardDeviation(final AngularSpeedTriad result) {
        mAccumulatedEstimator.getStandardDeviationTriad(result);
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
        return mAngularSpeedStandardDeviation;
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
        return new AngularSpeed(mAngularSpeedStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
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
        result.setValue(mAngularSpeedStandardDeviation);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets gyroscope base noise level PSD (Power Spectral Density)
     * expressed in (rad^2/s).
     *
     * @return gyroscope base noise level PSD.
     */
    public double getGyroscopeBaseNoiseLevelPsd() {
        return mAngularSpeedNoiseRootPsd * mAngularSpeedNoiseRootPsd;
    }

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    @Override
    public double getGyroscopeBaseNoiseLevelRootPsd() {
        return mAngularSpeedNoiseRootPsd;
    }

    /**
     * Post process provided input sample.
     *
     * @param sample an input sample.
     * @throws LockedException if generator is busy.
     */
    @Override
    protected void postProcess(final TimedBodyKinematics sample) throws LockedException {
        final TriadStaticIntervalDetector.Status status = mStaticIntervalDetector.getStatus();

        if (status == TriadStaticIntervalDetector.Status.INITIALIZING) {
            sample.getKinematics().getAngularRateTriad(mAngularSpeedTriad);
            mAccumulatedEstimator.addTriad(mAngularSpeedTriad);
        }

        // while we are in a dynamic interval, we must record all timed kinematics
        // along with accelerometer and gyroscope standard deviations
        if (status == TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL) {
            if (isDynamicIntervalSkipped()) {
                // dynamic interval has been skipped because there were too many
                // items in the sequence.
                mCurrentSequenceItems = null;
            } else {
                if (mPreviousStatus == TriadStaticIntervalDetector.Status.STATIC_INTERVAL) {
                    mPreviousAvgX = mStaticIntervalDetector.getAccumulatedAvgX();
                    mPreviousAvgY = mStaticIntervalDetector.getAccumulatedAvgY();
                    mPreviousAvgZ = mStaticIntervalDetector.getAccumulatedAvgZ();
                }

                addSequenceItem(sample);
            }
        } else if (status == TriadStaticIntervalDetector.Status.STATIC_INTERVAL
                && mPreviousStatus == TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL
                && mCurrentSequenceItems != null && !mCurrentSequenceItems.isEmpty()) {

            mCurrentAvgX = mStaticIntervalDetector.getInstantaneousAvgX();
            mCurrentAvgY = mStaticIntervalDetector.getInstantaneousAvgY();
            mCurrentAvgZ = mStaticIntervalDetector.getInstantaneousAvgZ();

            // we have all required data to generate a sequence
            BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = null;
            if (mListener != null) {
                sequence = new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(mPreviousAvgX, mPreviousAvgY, mPreviousAvgZ);
                sequence.setItems(mCurrentSequenceItems);
                sequence.setAfterMeanSpecificForceCoordinates(mCurrentAvgX, mCurrentAvgY, mCurrentAvgZ);
            }

            mCurrentSequenceItems = null;

            if (mListener != null) {
                mListener.onGeneratedMeasurement(this, sequence);
            }
        }
    }

    /**
     * Gets corresponding acceleration triad from provided input sample.
     * This method must store the result into {@link #mTriad}.
     *
     * @param sample input sample.
     */
    @Override
    protected void getAccelerationTriadFromInputSample(final TimedBodyKinematics sample) {
        sample.getKinematics().getSpecificForceTriad(mTriad);
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
        mPreviousStatus = TriadStaticIntervalDetector.Status.STATIC_INTERVAL;
    }

    /**
     * Handles a dynamic-to-static interval change.
     */
    @Override
    protected void handleDynamicToStaticChange() {
        mPreviousStatus = TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL;
    }

    /**
     * Handles an initialization completion.
     */
    @Override
    protected void handleInitializationCompleted() {
        mAccelerationStandardDeviation = mStaticIntervalDetector.getBaseNoiseLevel();
        mAngularSpeedStandardDeviation = mAccumulatedEstimator.getStandardDeviationNorm();
        mAngularSpeedNoiseRootPsd = mAccumulatedEstimator.getNoiseRootPsdNorm();

        mPreviousStatus = mStaticIntervalDetector.getStatus();
    }

    /**
     * Handles an error during initialization.
     */
    @Override
    protected void handleInitializationFailed() {
        mPreviousStatus = null;

        try {
            mAccumulatedEstimator.reset();
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
        if (mCurrentSequenceItems == null) {
            mCurrentSequenceItems = new ArrayList<>();
        }

        final BodyKinematics kinematics = new BodyKinematics(sample.getKinematics());
        final double timestampSeconds = sample.getTimestampSeconds();
        final StandardDeviationTimedBodyKinematics stdTimedKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds, mAccelerationStandardDeviation,
                        mAngularSpeedStandardDeviation);
        mCurrentSequenceItems.add(stdTimedKinematics);
    }
}
