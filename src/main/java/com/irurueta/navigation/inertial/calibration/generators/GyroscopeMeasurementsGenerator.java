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

import java.util.ArrayDeque;
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
 * <p>
 * Sequences are generated following the rules of Tedaldi et al. (ICRA 2014) / imu_tk:
 * <ul>
 *     <li>A sequence needs a valid static interval (at least {@link #getMinStaticSamples()} samples) both
 *     before and after it. The average specific force of each of them is used as the "before" and "after"
 *     gravity vectors. The initialization period is static, so its average is the first static average.</li>
 *     <li>If a static interval is too short to be valid, the dynamic intervals on both of its sides are
 *     joined into a single sequence.</li>
 *     <li>A sequence is generated when the static interval that follows it ends, so that the "after"
 *     vector is the average of the entire static interval. When no more samples are going to be processed,
 *     {@link #flush()} generates the sequence that is still pending.</li>
 *     <li>Items of a sequence are aligned with the samples that the (centred window) detector classified as
 *     dynamic, taking into account {@link #getDetectionDelaySamples()}.</li>
 *     <li>Generated sequences contain a gravity direction standard deviation
 *     ({@link BodyKinematicsSequence#getGravityDirectionStandardDeviation()}) computed from the standard
 *     deviation of the averaged static specific forces (noise divided by the square root of the number of
 *     averaged samples and by gravity norm) combined with the integrated gyroscope noise (root PSD multiplied by
 *     the square root of the sequence duration).</li>
 * </ul>
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
     * Items of the sequence that is being collected. A sequence starts with the first dynamic
     * sample after a valid static interval and finishes when the next valid static interval ends.
     * It is null when no sequence is being collected.
     */
    private List<StandardDeviationTimedBodyKinematics> currentSequenceItems;

    /**
     * Items of the static interval that is being processed. They are discarded if the static interval
     * turns out to be valid (and hence is used as gravity reference) or appended to the current sequence
     * if the static interval is too short and neighbouring dynamic intervals must be joined.
     */
    private List<StandardDeviationTimedBodyKinematics> currentStaticItems;

    /**
     * Samples (already converted to items) that have been received but not yet classified by the
     * detector, which classifies the sample at the centre of its window.
     */
    private final ArrayDeque<StandardDeviationTimedBodyKinematics> pendingItems = new ArrayDeque<>();

    /**
     * Indicates that the sample that completed initialization has not been processed yet.
     */
    private boolean initializationSamplePending;

    /**
     * Number of samples used to compute the average specific force of the static interval before the sequence.
     */
    private int previousAvgSamples;

    /**
     * Sum of x-coordinate of specific force of the samples of the static interval being processed,
     * expressed in meters per squared second (m/s^2).
     */
    private double currentStaticSumX;

    /**
     * Sum of y-coordinate of specific force of the samples of the static interval being processed,
     * expressed in meters per squared second (m/s^2).
     */
    private double currentStaticSumY;

    /**
     * Sum of z-coordinate of specific force of the samples of the static interval being processed,
     * expressed in meters per squared second (m/s^2).
     */
    private double currentStaticSumZ;

    /**
     * Number of samples of the static interval being processed.
     */
    private int currentStaticSamples;

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
        currentStaticItems = null;
        pendingItems.clear();
        initializationSamplePending = false;
        previousAvgSamples = 0;
        resetCurrentStatic();

        accelerationStandardDeviation = 0.0;
        angularSpeedStandardDeviation = 0.0;

        previousAvgX = null;
        previousAvgY = null;
        previousAvgZ = null;

        accumulatedEstimator.reset();

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
     * Detector classifies the sample at the centre of its window, so samples are buffered
     * until they are classified in order to align items with detected intervals.
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
            return;
        }

        if (status == TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED && initializationSamplePending) {
            // sample that completed initialization belongs to the initialization period
            initializationSamplePending = false;
            return;
        }

        if (status != TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED
                && status != TriadStaticIntervalDetector.Status.STATIC_INTERVAL
                && status != TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL) {
            return;
        }

        pendingItems.addLast(createItem(sample));

        if (status == TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED) {
            // first classified sample has not been reached yet
            return;
        }

        // oldest pending sample is the one that has just been classified
        final var item = pendingItems.pollFirst();

        if (status == TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL) {
            if (isDynamicIntervalSkipped()) {
                // dynamic interval has been skipped because there were too many
                // items in the sequence. Gravity reference is lost.
                currentSequenceItems = null;
                currentStaticItems = null;
                previousAvgX = null;
                previousAvgY = null;
                previousAvgZ = null;
            } else if (currentSequenceItems != null) {
                currentSequenceItems.add(item);
            }
        } else if (currentSequenceItems != null) {
            // static interval, which might be too short to be valid and then be joined to the sequence
            if (currentStaticItems == null) {
                currentStaticItems = new ArrayList<>();
            }
            currentStaticItems.add(item);

            final var kinematics = item.getKinematics();
            currentStaticSumX += kinematics.getFx();
            currentStaticSumY += kinematics.getFy();
            currentStaticSumZ += kinematics.getFz();
            currentStaticSamples++;
        }
    }

    /**
     * Generates the sequence that is pending, if any, without waiting for the end of the static interval
     * that follows it.
     * A sequence is normally generated when the static interval that follows it ends, so that the
     * "after" gravity vector is the average of the entire static interval. Consequently, when no more
     * samples are going to be processed (e.g. a recording ends while the device remains static), the last
     * sequence would never be generated. This method generates it using the samples of the current static
     * interval that have been classified so far, as long as there are at least
     * {@link #getMinStaticSamples()} of them.
     * If more samples are processed afterwards, the sequence is not generated again.
     *
     * @return true if a sequence has been generated, false otherwise.
     * @throws LockedException if generator is busy.
     */
    public boolean flush() throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }

        if (listener == null || currentSequenceItems == null || currentSequenceItems.isEmpty()
                || previousAvgX == null || currentStaticSamples < Math.max(getMinStaticSamples(), 1)
                || staticIntervalDetector.getStatus() != TriadStaticIntervalDetector.Status.STATIC_INTERVAL) {
            return false;
        }

        emitSequence(currentStaticSumX / currentStaticSamples, currentStaticSumY / currentStaticSamples,
                currentStaticSumZ / currentStaticSamples, currentStaticSamples);
        currentStaticItems = null;
        return true;
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
     * Handles a static-to-dynamic interval change, which means that a static interval has ended.
     * If such static interval is valid, its average is the gravity reference of the sequence that
     * finishes now (which is generated) and of the sequence that starts now. If the static interval is too
     * short, the neighbouring dynamic intervals are joined into the same sequence.
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
        // the interval that starts with the initialization period is always valid because the
        // initialization period is long enough and its average is the first static average
        final var valid = isFirstStaticInterval() || !isStaticIntervalSkipped();
        final var samples = Math.max(getAccumulatedStaticSamples(), 1);

        if (valid) {
            if (currentSequenceItems != null && !currentSequenceItems.isEmpty() && previousAvgX != null) {
                // static interval is valid so sequence is complete
                emitSequence(accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ, samples);
            }

            // static interval is the gravity reference ("before" vector) of next sequence
            previousAvgX = accumulatedAvgX;
            previousAvgY = accumulatedAvgY;
            previousAvgZ = accumulatedAvgZ;
            previousAvgSamples = samples;
            currentSequenceItems = new ArrayList<>();
        } else if (currentSequenceItems != null && currentStaticItems != null) {
            // static interval is too short: neighbouring dynamic intervals are joined
            currentSequenceItems.addAll(currentStaticItems);
        }
        // when there is no previous valid static interval, no sequence is being collected, and
        // items are discarded until a valid static interval is found

        currentStaticItems = null;
        resetCurrentStatic();
    }

    /**
     * Handles a dynamic-to-static interval change.
     */
    @Override
    protected void handleDynamicToStaticChange() {
        currentStaticItems = null;
        resetCurrentStatic();
    }

    /**
     * Handles an initialization completion.
     */
    @Override
    protected void handleInitializationCompleted() {
        // base noise level is variance based, so the standard deviation is recovered from the root PSD
        final var timeInterval = staticIntervalDetector.getTimeInterval();
        accelerationStandardDeviation = timeInterval > 0.0
                ? staticIntervalDetector.getBaseNoiseLevelRootPsd() / Math.sqrt(timeInterval) : 0.0;
        angularSpeedStandardDeviation = accumulatedEstimator.getStandardDeviationNorm();
        angularSpeedNoiseRootPsd = accumulatedEstimator.getNoiseRootPsdNorm();

        currentSequenceItems = null;
        currentStaticItems = null;
        pendingItems.clear();
        initializationSamplePending = true;
        resetCurrentStatic();
    }

    /**
     * Handles an error during initialization.
     */
    @Override
    protected void handleInitializationFailed() {
        currentSequenceItems = null;
        currentStaticItems = null;
        pendingItems.clear();
        initializationSamplePending = false;

        try {
            accumulatedEstimator.reset();
        } catch (final LockedException ignore) {
            // no action needed
        }
    }

    /**
     * Resets accumulated specific force of the static interval being processed.
     */
    private void resetCurrentStatic() {
        currentStaticSumX = 0.0;
        currentStaticSumY = 0.0;
        currentStaticSumZ = 0.0;
        currentStaticSamples = 0;
    }

    /**
     * Creates a sequence item from provided sample.
     *
     * @param sample sample to generate a sequence item from.
     * @return created item.
     */
    private StandardDeviationTimedBodyKinematics createItem(final TimedBodyKinematics sample) {
        final var kinematics = new BodyKinematics(sample.getKinematics());
        final var timestampSeconds = sample.getTimestampSeconds();
        return new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds,
                accelerationStandardDeviation, angularSpeedStandardDeviation);
    }

    /**
     * Generates a sequence with current items, whose "after" gravity vector is the average of the whole
     * static interval that has just ended.
     *
     * @param afterAvgX x-coordinate of the average specific force of the static interval after the sequence.
     * @param afterAvgY y-coordinate of the average specific force of the static interval after the sequence.
     * @param afterAvgZ z-coordinate of the average specific force of the static interval after the sequence.
     * @param afterSamples number of samples of the static interval after the sequence.
     */
    private void emitSequence(final double afterAvgX, final double afterAvgY, final double afterAvgZ,
                              final int afterSamples) {
        if (listener == null) {
            return;
        }

        final var items = currentSequenceItems;
        final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
        sequence.setBeforeMeanSpecificForceCoordinates(previousAvgX, previousAvgY, previousAvgZ);
        sequence.setItems(items);
        sequence.setAfterMeanSpecificForceCoordinates(afterAvgX, afterAvgY, afterAvgZ);

        // standard deviation of gravity direction: uncertainty of the mean specific force of the static periods
        // before and after (mean noise divided by gravity norm), combined with the uncertainty of the integrated
        // gyroscope noise (root PSD multiplied by square root of sequence duration)
        final var beforeNorm = Math.sqrt(previousAvgX * previousAvgX + previousAvgY * previousAvgY
                + previousAvgZ * previousAvgZ);
        final var afterNorm = Math.sqrt(afterAvgX * afterAvgX + afterAvgY * afterAvgY + afterAvgZ * afterAvgZ);
        // accelerationStandardDeviation and angularSpeedNoiseRootPsd are norms of the three coordinates, so
        // they are converted to per-coordinate variances, since the result is a per-component standard deviation
        final var accelerationVariance = accelerationStandardDeviation * accelerationStandardDeviation
                / BodyKinematics.COMPONENTS;
        final var beforeVariance = beforeNorm > 0.0
                ? accelerationVariance / (previousAvgSamples * beforeNorm * beforeNorm) : 0.0;
        final var afterVariance = afterNorm > 0.0
                ? accelerationVariance / (afterSamples * afterNorm * afterNorm) : 0.0;
        final var duration = Math.max(
                items.get(items.size() - 1).getTimestampSeconds() - items.get(0).getTimestampSeconds(), 0.0);
        final var gyroVariance = angularSpeedNoiseRootPsd * angularSpeedNoiseRootPsd * duration
                / BodyKinematics.COMPONENTS;
        sequence.setGravityDirectionStandardDeviation(Math.sqrt(beforeVariance + afterVariance + gyroVariance));

        currentSequenceItems = null;

        listener.onGeneratedMeasurement(this, sequence);
    }
}
