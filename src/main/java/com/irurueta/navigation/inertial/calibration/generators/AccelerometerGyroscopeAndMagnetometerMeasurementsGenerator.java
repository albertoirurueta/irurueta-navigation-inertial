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
import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.GyroscopeNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Generates measurements for the calibration of accelerometers and gyroscopes by alternating
 * static and dynamic intervals where device is kept static or moved.
 * Generated measurements must be used with accelerometer calibrators based
 * on the knowledge of gravity norm (or Earth position) when the device orientation
 * is unknown, with easy gyroscope calibrators and with magnetometer calibrators based
 * on the knowledge of position on Earth and time instant.
 * Notice that accuracy of the gyroscope calibration is very sensitive to the
 * accuracy of detected dynamic intervals respect the average specific forces
 * during static intervals.
 * In order to increase the accuracy, calibration should be repeated trying different
 * threshold factors {@link #getThresholdFactor()}.
 *
 * @see AccelerometerMeasurementsGenerator
 * @see GyroscopeMeasurementsGenerator
 * @see MagnetometerMeasurementsGenerator
 */
public class AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator implements
        AccelerometerNoiseRootPsdSource, GyroscopeNoiseRootPsdSource {

    /**
     * Listener to handle generated events.
     */
    private AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener listener;

    /**
     * Listener for internal accelerometer measurements generator.
     */
    private final AccelerometerMeasurementsGeneratorListener accelerometerListener =
            new AccelerometerMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onInitializationCompleted(
                        final AccelerometerMeasurementsGenerator generator, final double baseNoiseLevel) {
                    // no action required
                }

                @Override
                public void onError(
                        final AccelerometerMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    if (listener != null) {
                        listener.onError(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this, reason);
                    }
                }

                @Override
                public void onStaticIntervalDetected(final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onDynamicIntervalDetected(final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onStaticIntervalSkipped(final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onDynamicIntervalSkipped(final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onGeneratedMeasurement(
                        final AccelerometerMeasurementsGenerator generator,
                        final StandardDeviationBodyKinematics measurement) {
                    if (listener != null) {
                        listener.onGeneratedAccelerometerMeasurement(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this,
                                measurement);
                    }
                }

                @Override
                public void onReset(final AccelerometerMeasurementsGenerator generator) {
                    // no action required
                }
            };

    /**
     * Listener for internal gyroscope measurements generator.
     */
    private final GyroscopeMeasurementsGeneratorListener gyroscopeListener =
            new GyroscopeMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onInitializationCompleted(
                        final GyroscopeMeasurementsGenerator generator, final double baseNoiseLevel) {
                    // no action required
                }

                @Override
                public void onError(
                        final GyroscopeMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    // no action required
                }

                @Override
                public void onStaticIntervalDetected(final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onDynamicIntervalDetected(final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onStaticIntervalSkipped(final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onDynamicIntervalSkipped(final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }

                @Override
                public void onGeneratedMeasurement(
                        final GyroscopeMeasurementsGenerator generator,
                        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
                    if (listener != null) {
                        listener.onGeneratedGyroscopeMeasurement(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this, measurement);
                    }
                }

                @Override
                public void onReset(final GyroscopeMeasurementsGenerator generator) {
                    // no action required
                }
            };

    /**
     * Listener for internal magnetometer measurements generator.
     */
    private final MagnetometerMeasurementsGeneratorListener magnetometerListener =
            new MagnetometerMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(final MagnetometerMeasurementsGenerator generator) {
                    if (listener != null) {
                        listener.onInitializationStarted(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
                    }
                }

                @Override
                public void onInitializationCompleted(
                        final MagnetometerMeasurementsGenerator generator, final double baseNoiseLevel) {
                    if (listener != null) {
                        listener.onInitializationCompleted(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this,
                                baseNoiseLevel);
                    }
                }

                @Override
                public void onError(final MagnetometerMeasurementsGenerator generator,
                                    final TriadStaticIntervalDetector.ErrorReason reason) {
                    // no action required
                }

                @Override
                public void onStaticIntervalDetected(final MagnetometerMeasurementsGenerator generator) {
                    if (listener != null) {
                        listener.onStaticIntervalDetected(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
                    }
                }

                @Override
                public void onDynamicIntervalDetected(final MagnetometerMeasurementsGenerator generator) {
                    if (listener != null) {
                        listener.onDynamicIntervalDetected(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
                    }
                }

                @Override
                public void onStaticIntervalSkipped(final MagnetometerMeasurementsGenerator generator) {
                    if (listener != null) {
                        listener.onStaticIntervalSkipped(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
                    }
                }

                @Override
                public void onDynamicIntervalSkipped(final MagnetometerMeasurementsGenerator generator) {
                    if (listener != null) {
                        listener.onDynamicIntervalSkipped(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this);
                    }
                }

                @Override
                public void onGeneratedMeasurement(
                        final MagnetometerMeasurementsGenerator generator,
                        final StandardDeviationBodyMagneticFluxDensity measurement) {
                    if (listener != null) {
                        listener.onGeneratedMagnetometerMeasurement(
                                AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator.this, measurement);
                    }
                }

                @Override
                public void onReset(final MagnetometerMeasurementsGenerator generator) {
                    // no action required
                }
            };

    /**
     * Internal accelerometer measurements generator.
     */
    private final AccelerometerMeasurementsGenerator accelerometerMeasurementsGenerator =
            new AccelerometerMeasurementsGenerator(accelerometerListener);

    /**
     * Internal gyroscope measurements generator.
     */
    private final GyroscopeMeasurementsGenerator gyroscopeMeasurementsGenerator =
            new GyroscopeMeasurementsGenerator(gyroscopeListener);

    /**
     * Internal magnetometer measurements generator.
     */
    private final MagnetometerMeasurementsGenerator magnetometerMeasurementsGenerator =
            new MagnetometerMeasurementsGenerator(magnetometerListener);

    /**
     * Indicates whether generator is running or not.
     */
    private boolean running;

    /**
     * Timed body kinematics instance to be reused.
     */
    private final TimedBodyKinematics timedKinematics = new TimedBodyKinematics();

    /**
     * Constructor.
     */
    public AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this generator.
     */
    public AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener listener) {
        this();
        this.listener = listener;
    }

    /**
     * Gets time interval between input samples expressed in seconds (s).
     *
     * @return time interval between input samples.
     */
    public double getTimeInterval() {
        return accelerometerMeasurementsGenerator.getTimeInterval();
    }

    /**
     * Sets time interval between input samples expressed in seconds (s).
     *
     * @param timeInterval time interval between input samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if generator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        accelerometerMeasurementsGenerator.setTimeInterval(timeInterval);
        gyroscopeMeasurementsGenerator.setTimeInterval(timeInterval);
        magnetometerMeasurementsGenerator.setTimeInterval(timeInterval);
    }

    /**
     * Gets time interval between input samples.
     *
     * @return time interval between input samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(getTimeInterval(), TimeUnit.SECOND);
    }

    /**
     * Gets time interval between input samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(getTimeInterval());
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between input samples.
     *
     * @param timeInterval time interval between input samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(TimeConverter.convert(timeInterval.getValue().doubleValue(), timeInterval.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Gets minimum number of samples required in a static interval to be taken into account.
     * Smaller static intervals will be discarded.
     *
     * @return minimum number of samples required in a static interval to be taken into account.
     */
    public int getMinStaticSamples() {
        return accelerometerMeasurementsGenerator.getMinStaticSamples();
    }

    /**
     * Sets minimum number of samples required in a static interval to be taken into account.
     * Smaller static intervals will be discarded.
     *
     * @param minStaticSamples minimum number of samples required in a static interval to be
     *                         taken into account.
     * @throws LockedException          if generator is busy.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setMinStaticSamples(final int minStaticSamples) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.setMinStaticSamples(minStaticSamples);
        gyroscopeMeasurementsGenerator.setMinStaticSamples(minStaticSamples);
        magnetometerMeasurementsGenerator.setMinStaticSamples(minStaticSamples);
    }

    /**
     * Gets maximum number of samples allowed in dynamic intervals.
     *
     * @return maximum number of samples allowed in dynamic intervals.
     */
    public int getMaxDynamicSamples() {
        return accelerometerMeasurementsGenerator.getMaxDynamicSamples();
    }

    /**
     * Sets maximum number of samples allowed in dynamic intervals.
     *
     * @param maxDynamicSamples maximum number of samples allowed in dynamic intervals.
     * @throws LockedException          if generator is busy.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setMaxDynamicSamples(final int maxDynamicSamples) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.setMaxDynamicSamples(maxDynamicSamples);
        gyroscopeMeasurementsGenerator.setMaxDynamicSamples(maxDynamicSamples);
        magnetometerMeasurementsGenerator.setMaxDynamicSamples(maxDynamicSamples);
    }

    /**
     * Gets listener to handle generated events.
     *
     * @return listener to handle generated events.
     */
    public AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle generated events.
     *
     * @param listener listener to handle generated events.
     * @throws LockedException if generator is busy.
     */
    public void setListener(final AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener listener)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets length of number of samples to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     *
     * @return length of number of samples to keep within the window.
     */
    public int getWindowSize() {
        return accelerometerMeasurementsGenerator.getWindowSize();
    }

    /**
     * Sets length of number of samples to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     * Window size must always be larger than allowed minimum value, which is 2 and
     * must have an odd value.
     *
     * @param windowSize length of number of samples to keep within the window.
     * @throws LockedException          if detector is busy processing a previous sample.
     * @throws IllegalArgumentException if provided value is not valid.
     */
    public void setWindowSize(final int windowSize) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.setWindowSize(windowSize);
        gyroscopeMeasurementsGenerator.setWindowSize(windowSize);
        magnetometerMeasurementsGenerator.setWindowSize(windowSize);
    }

    /**
     * Gets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @return number of samples to be processed initially.
     */
    public int getInitialStaticSamples() {
        return accelerometerMeasurementsGenerator.getInitialStaticSamples();
    }

    /**
     * Sets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @param initialStaticSamples number of samples to be processed initially.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is less than
     *                                  {@link TriadStaticIntervalDetector#MINIMUM_INITIAL_STATIC_SAMPLES}
     */
    public void setInitialStaticSamples(final int initialStaticSamples) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.setInitialStaticSamples(initialStaticSamples);
        gyroscopeMeasurementsGenerator.setInitialStaticSamples(initialStaticSamples);
        magnetometerMeasurementsGenerator.setInitialStaticSamples(initialStaticSamples);
    }

    /**
     * Gets factor to be applied to detected base noise level in order to
     * determine threshold for static/dynamic period changes. This factor is
     * unit-less.
     *
     * @return factor to be applied to detected base noise level.
     */
    public double getThresholdFactor() {
        return accelerometerMeasurementsGenerator.getThresholdFactor();
    }

    /**
     * Sets factor to be applied to detected base noise level in order to
     * determine threshold for static/dynamic period changes. This factor is
     * unit-less.
     *
     * @param thresholdFactor factor to be applied to detected base noise level.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setThresholdFactor(final double thresholdFactor) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.setThresholdFactor(thresholdFactor);
        gyroscopeMeasurementsGenerator.setThresholdFactor(thresholdFactor);
        magnetometerMeasurementsGenerator.setThresholdFactor(thresholdFactor);
    }

    /**
     * Gets factor to determine that a sudden movement has occurred during
     * initialization if instantaneous noise level exceeds accumulated noise
     * level by this factor amount.
     * This factor is unit-less.
     *
     * @return factor to determine that a sudden movement has occurred.
     */
    public double getInstantaneousNoiseLevelFactor() {
        return accelerometerMeasurementsGenerator.getInstantaneousNoiseLevelFactor();
    }

    /**
     * Sets factor to determine that a sudden movement has occurred during
     * initialization if instantaneous noise level exceeds accumulated noise
     * level by this factor amount.
     * This factor is unit-less.
     *
     * @param instantaneousNoiseLevelFactor factor to determine that a sudden
     *                                      movement has occurred during
     *                                      initialization.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setInstantaneousNoiseLevelFactor(final double instantaneousNoiseLevelFactor) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.setInstantaneousNoiseLevelFactor(instantaneousNoiseLevelFactor);
        gyroscopeMeasurementsGenerator.setInstantaneousNoiseLevelFactor(instantaneousNoiseLevelFactor);
        magnetometerMeasurementsGenerator.setInstantaneousNoiseLevelFactor(instantaneousNoiseLevelFactor);
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2).
     *
     * @return overall absolute threshold to determine whether there has
     * been excessive motion.
     */
    public double getBaseNoiseLevelAbsoluteThreshold() {
        return accelerometerMeasurementsGenerator.getBaseNoiseLevelAbsoluteThreshold();
    }

    /**
     * Sets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2).
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(final double baseNoiseLevelAbsoluteThreshold)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);
        gyroscopeMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);
        magnetometerMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     *
     * @return overall absolute threshold to determine whether there has been
     * excessive motion.
     */
    public Acceleration getBaseNoiseLevelAbsoluteThresholdAsMeasurement() {
        return accelerometerMeasurementsGenerator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    public void getBaseNoiseLevelAbsoluteThresholdAsMeasurement(final Acceleration result) {
        accelerometerMeasurementsGenerator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result);
    }

    /**
     * Sets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(final Acceleration baseNoiseLevelAbsoluteThreshold)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);
        gyroscopeMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);
        magnetometerMeasurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets internal status of this generator.
     *
     * @return internal status of this generator.
     */
    public TriadStaticIntervalDetector.Status getStatus() {
        return accelerometerMeasurementsGenerator.getStatus();
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization expressed in meters per squared second (m/s^2).
     * This is equal to the standard deviation of the accelerometer measurements
     * during initialization phase.
     *
     * @return accelerometer base noise level.
     */
    public double getAccelerometerBaseNoiseLevel() {
        return accelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevel();
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization.
     * This is equal to the standard deviation of the accelerometer measurements
     * during initialization phase.
     *
     * @return measurement base noise level.
     */
    public Acceleration getAccelerometerBaseNoiseLevelAsMeasurement() {
        return accelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevelAsMeasurement();
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerometerBaseNoiseLevelAsMeasurement(final Acceleration result) {
        accelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevelAsMeasurement(result);
    }

    /**
     * Gets accelerometer base noise level PSD (Power Spectral Density)
     * expressed in (m^2 * s^-3).
     *
     * @return accelerometer base noise level PSD.
     */
    public double getAccelerometerBaseNoiseLevelPsd() {
        return accelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevelPsd();
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    @Override
    public double getAccelerometerBaseNoiseLevelRootPsd() {
        return accelerometerMeasurementsGenerator.getAccelerometerBaseNoiseLevelRootPsd();
    }

    /**
     * Gets threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2).
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public double getThreshold() {
        return accelerometerMeasurementsGenerator.getThreshold();
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public Acceleration getThresholdAsMeasurement() {
        return accelerometerMeasurementsGenerator.getThresholdAsMeasurement();
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @param result instance where result will be stored.
     */
    public void getThresholdAsMeasurement(final Acceleration result) {
        accelerometerMeasurementsGenerator.getThresholdAsMeasurement(result);
    }

    /**
     * Gets number of samples that have been processed in a static period so far.
     *
     * @return number of samples that have been processed in a static period so far.
     */
    public int getProcessedStaticSamples() {
        return accelerometerMeasurementsGenerator.getProcessedStaticSamples();
    }

    /**
     * Gets number of samples that have been processed in a dynamic period so far.
     *
     * @return number of samples that have been processed in a dynamic period so far.
     */
    public int getProcessedDynamicSamples() {
        return accelerometerMeasurementsGenerator.getProcessedDynamicSamples();
    }

    /**
     * Indicates whether last static interval must be skipped.
     *
     * @return true if last static interval must be skipped.
     */
    public boolean isStaticIntervalSkipped() {
        return accelerometerMeasurementsGenerator.isStaticIntervalSkipped();
    }

    /**
     * Indicates whether last dynamic interval must be skipped.
     *
     * @return true if last dynamic interval must be skipped.
     */
    public boolean isDynamicIntervalSkipped() {
        return accelerometerMeasurementsGenerator.isDynamicIntervalSkipped();
    }

    /**
     * Indicates whether generator is running or not.
     *
     * @return true if generator is running, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Processes a sample of data.
     *
     * @param sample sample of data to be processed.
     * @return true if provided samples has been processed, false if provided triad has been skipped because
     * generator previously failed. If generator previously failed, it will need to be reset before
     * processing additional samples.
     * @throws LockedException if generator is busy processing a previous sample.
     */
    public boolean process(final TimedBodyKinematicsAndMagneticFluxDensity sample) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        try {
            running = true;

            sample.getTimedKinematics(timedKinematics);

            return accelerometerMeasurementsGenerator.process(sample.getKinematics())
                    && gyroscopeMeasurementsGenerator.process(timedKinematics)
                    && magnetometerMeasurementsGenerator.process(sample);
        } finally {
            running = false;
        }
    }

    /**
     * Resets this generator.
     *
     * @throws LockedException if generator is busy.
     */
    public void reset() throws LockedException {
        if (running) {
            throw new LockedException();
        }

        accelerometerMeasurementsGenerator.reset();
        gyroscopeMeasurementsGenerator.reset();
        magnetometerMeasurementsGenerator.reset();

        if (listener != null) {
            listener.onReset(this);
        }
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @return estimated average angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAvgAngularSpeedTriad() {
        return gyroscopeMeasurementsGenerator.getInitialAvgAngularSpeedTriad();
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAvgAngularSpeedTriad(final AngularSpeedTriad result) {
        gyroscopeMeasurementsGenerator.getInitialAvgAngularSpeedTriad(result);
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @return estimated standard deviation of angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAngularSpeedTriadStandardDeviation() {
        return gyroscopeMeasurementsGenerator.getInitialAngularSpeedTriadStandardDeviation();
    }

    /**
     * Gets estimated standard deviation of angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAngularSpeedTriadStandardDeviation(final AngularSpeedTriad result) {
        gyroscopeMeasurementsGenerator.getInitialAngularSpeedTriadStandardDeviation(result);
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
        return gyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevel();
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
        return gyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevelAsMeasurement();
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
        gyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevelAsMeasurement(result);
    }

    /**
     * Gets gyroscope base noise level PSD (Power Spectral Density)
     * expressed in (rad^2/s).
     *
     * @return gyroscope base noise level PSD.
     */
    public double getGyroscopeBaseNoiseLevelPsd() {
        return gyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevelPsd();
    }

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    @Override
    public double getGyroscopeBaseNoiseLevelRootPsd() {
        return gyroscopeMeasurementsGenerator.getGyroscopeBaseNoiseLevelRootPsd();
    }
}
