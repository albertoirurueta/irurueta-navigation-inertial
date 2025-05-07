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
package com.irurueta.navigation.inertial.calibration.intervals;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.Triad;
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedTriadNoiseEstimator;
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator;
import com.irurueta.units.Measurement;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Abstract base class for detectors in charge of determining when a static period of
 * measurements starts and finishes.
 * Static periods are periods of time where the device is considered to
 * remain static (no movement applied to it).
 *
 * @param <U> type of unit.
 * @param <M> a type of measurement.
 * @param <T> a triad type.
 * @param <D> a detector type.
 * @param <L> a listener type.
 */
public abstract class TriadStaticIntervalDetector<U extends Enum<?>, M extends Measurement<U>,
        T extends Triad<U, M>, D extends TriadStaticIntervalDetector<U, M, T, D, L>,
        L extends TriadStaticIntervalDetectorListener<U, M, T, D>> {

    /**
     * Number of samples to keep within the window by default.
     * For a sensor generating 100 samples/second, this is equivalent to 1 second.
     * For a sensor generating 50 samples/second, this is equivalent to 2 seconds.
     */
    public static final int DEFAULT_WINDOW_SIZE = WindowedTriadNoiseEstimator.DEFAULT_WINDOW_SIZE;

    /**
     * Number of samples to process during the initial static period to determine the sensor
     * (accelerometer, gyroscope or magnetometer) noise level.
     * For a sensor generating 100 samples/second, this is equivalent to 50 seconds.
     * For a sensor generating 50 samples/second, this is equivalent to 100 seconds.
     */
    public static final int DEFAULT_INITIAL_STATIC_SAMPLES = 5000;

    /**
     * Minimum allowed number of samples to be processed during the initial static period.
     */
    public static final int MINIMUM_INITIAL_STATIC_SAMPLES = 2;

    /**
     * Default factor to be applied to detected base noise level in order to determine
     * threshold for static/dynamic period changes. This factor is unit-less.
     */
    public static final double DEFAULT_THRESHOLD_FACTOR = 2.0;

    /**
     * Default factor to determine that a sudden movement has occurred during initialization
     * if instantaneous noise level exceeds accumulated noise level by this factor amount.
     * This factor is unit-less.
     */
    public static final double DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR = 2.0;

    /**
     * Default overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase.
     * This threshold is expressed in meters per squared second (m/s^2) for acceleration, radians
     * per second (rad/s) for angular speed or Teslas (T) for magnetic flux density, and by
     * default it is set to the maximum allowed value, thus effectively disabling this error
     * condition check during initialization.
     */
    public static final double DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = Double.MAX_VALUE;

    /**
     * Number of samples to keep in window to find instantaneous noise level averaged within
     * the window of samples.
     * Window size should contain about 1 or 2 seconds of data to be averaged to obtain
     * a more reliable instantaneous noise level.
     */
    private int windowSize = DEFAULT_WINDOW_SIZE;

    /**
     * Number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     */
    private int initialStaticSamples = DEFAULT_INITIAL_STATIC_SAMPLES;

    /**
     * Factor to be applied to detected base noise level in order to determine
     * threshold for static/dynamic period changes. This factor is unit-less.
     */
    private double thresholdFactor = DEFAULT_THRESHOLD_FACTOR;

    /**
     * Factor to determine that a sudden movement has occurred during initialization if
     * instantaneous noise level exceeds accumulated noise level by this factor amount.
     * This factor is unit-less.
     */
    private double instantaneousNoiseLevelFactor = DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR;

    /**
     * Overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this threshold when
     * initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
     */
    private double baseNoiseLevelAbsoluteThreshold = DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD;

    /**
     * Listener to handle events generated by this detector.
     */
    private L listener;

    /**
     * Current status of this detector.
     */
    private Status status = Status.IDLE;

    /**
     * Measurement base noise level that has been detected during initialization expressed in
     * meters per squared second (m/s^2) for acceleration, radians per second (rad/s) for
     * angular speed or Teslas (T) for magnetic flux density.
     */
    private double baseNoiseLevel;

    /**
     * Threshold to determine static/dynamic period changes expressed in meters per squared
     * second (m/s^2) for acceleration, radians per second (rad/s) for angular speed or
     * Teslas (T) for magnetic flux density.
     */
    private double threshold;

    /**
     * Indicates whether this detector is busy processing last provided sample.
     */
    private boolean running;

    /**
     * Number of samples that have been processed so far.
     */
    private int processedSamples;

    /**
     * Average x-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double accumulatedAvgX;

    /**
     * Average y-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double accumulatedAvgY;

    /**
     * Average z-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double accumulatedAvgZ;

    /**
     * Standard deviation of x-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double accumulatedStdX;

    /**
     * Standard deviation of y-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double accumulatedStdY;

    /**
     * Standard deviation of z-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     */
    private double accumulatedStdZ;

    /**
     * Windowed average x-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     */
    private double instantaneousAvgX;

    /**
     * Windowed average y-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     */
    private double instantaneousAvgY;

    /**
     * Windowed average z-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     */
    private double instantaneousAvgZ;

    /**
     * Windowed standard deviation of x-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas (T)
     * for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     */
    private double instantaneousStdX;

    /**
     * Windowed standard deviation of y-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas (T)
     * for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     */
    private double instantaneousStdY;

    /**
     * Windowed standard deviation of z-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas (T)
     * for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     */
    private double instantaneousStdZ;

    /**
     * Estimator to find instantaneous measurement noise level averaged for a certain window of samples.
     */
    private final WindowedTriadNoiseEstimator<U, M, T, ?, ?> windowedNoiseEstimator;

    /**
     * Estimator to find accumulated accelerometer noise level.
     */
    private final AccumulatedTriadNoiseEstimator<U, M, T, ?, ?> accumulatedNoiseEstimator;

    /**
     * Constructor.
     *
     * @param windowedNoiseEstimator    windowed noise estimator to estimate noise within a window of measures.
     * @param accumulatedNoiseEstimator accumulated noise estimator to estimate accumulated noise and average.
     */
    protected TriadStaticIntervalDetector(
            final WindowedTriadNoiseEstimator<U, M, T, ?, ?> windowedNoiseEstimator,
            final AccumulatedTriadNoiseEstimator<U, M, T, ?, ?> accumulatedNoiseEstimator) {
        this.windowedNoiseEstimator = windowedNoiseEstimator;
        this.accumulatedNoiseEstimator = accumulatedNoiseEstimator;
    }

    /**
     * Constructor.
     *
     * @param windowedNoiseEstimator    windowed noise estimator to estimate noise within a window of measures.
     * @param accumulatedNoiseEstimator accumulated noise estimator to estimate accumulated noise and average.
     * @param listener                  listener to handle events generated by this detector.
     */
    protected TriadStaticIntervalDetector(
            final WindowedTriadNoiseEstimator<U, M, T, ?, ?> windowedNoiseEstimator,
            final AccumulatedTriadNoiseEstimator<U, M, T, ?, ?> accumulatedNoiseEstimator, final L listener) {
        this(windowedNoiseEstimator, accumulatedNoiseEstimator);
        this.listener = listener;
    }

    /**
     * Gets length of number of samples to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     *
     * @return length of number of samples to keep within the window.
     */
    public int getWindowSize() {
        return windowSize;
    }

    /**
     * Sets length of number of samples to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     * Window size must always be larger than allowed minimum value, which is 2 and
     * must have and odd value.
     *
     * @param windowSize length of number of samples to keep within the window.
     * @throws LockedException          if detector is busy processing a previous sample.
     * @throws IllegalArgumentException if provided value is not valid.
     */
    public void setWindowSize(final int windowSize) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        windowedNoiseEstimator.setWindowSize(windowSize);
        this.windowSize = windowSize;
    }

    /**
     * Gets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @return number of samples to be processed initially.
     */
    public int getInitialStaticSamples() {
        return initialStaticSamples;
    }

    /**
     * Sets number of samples to be processed initially while keeping the sensor static in order
     * to find the base noise level when device is static.
     *
     * @param initialStaticSamples number of samples to be processed initially.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is less than {@link #MINIMUM_INITIAL_STATIC_SAMPLES}
     */
    public void setInitialStaticSamples(final int initialStaticSamples) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (initialStaticSamples < MINIMUM_INITIAL_STATIC_SAMPLES) {
            throw new IllegalArgumentException();
        }

        this.initialStaticSamples = initialStaticSamples;
    }

    /**
     * Gets factor to be applied to detected base noise level in order to
     * determine threshold for static/dynamic period changes. This factor is
     * unit-less.
     *
     * @return factor to be applied to detected base noise level.
     */
    public double getThresholdFactor() {
        return thresholdFactor;
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
        if (thresholdFactor <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.thresholdFactor = thresholdFactor;
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
        return instantaneousNoiseLevelFactor;
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
    public void setInstantaneousNoiseLevelFactor(
            final double instantaneousNoiseLevelFactor) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (instantaneousNoiseLevelFactor <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.instantaneousNoiseLevelFactor = instantaneousNoiseLevelFactor;
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     *
     * @return overall absolute threshold to determine whether there has
     * been excessive motion.
     */
    public double getBaseNoiseLevelAbsoluteThreshold() {
        return baseNoiseLevelAbsoluteThreshold;
    }

    /**
     * Sets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     * This threshold is expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if detector is busy.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(
            final double baseNoiseLevelAbsoluteThreshold) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (baseNoiseLevelAbsoluteThreshold <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.baseNoiseLevelAbsoluteThreshold = baseNoiseLevelAbsoluteThreshold;
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
    public M getBaseNoiseLevelAbsoluteThresholdAsMeasurement() {
        return createMeasurement(baseNoiseLevelAbsoluteThreshold, getDefaultUnit());
    }

    /**
     * Gets overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * Failure will be detected if estimated base noise level exceeds this
     * threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    public void getBaseNoiseLevelAbsoluteThresholdAsMeasurement(final M result) {
        result.setValue(baseNoiseLevelAbsoluteThreshold);
        result.setUnit(getDefaultUnit());
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
    public void setBaseNoiseLevelAbsoluteThreshold(
            final M baseNoiseLevelAbsoluteThreshold) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        setBaseNoiseLevelAbsoluteThreshold(convertMeasurement(baseNoiseLevelAbsoluteThreshold));
    }

    /**
     * Gets listener to handle events generated by this detector.
     *
     * @return listener to handle events.
     */
    public L getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events generated by this detector.
     *
     * @param listener listener to handle events.
     * @throws LockedException if detector is busy.
     */
    public void setListener(final L listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets time interval between triad samples expressed in seconds (s).
     *
     * @return time interval between triad samples.
     */
    public double getTimeInterval() {
        return windowedNoiseEstimator.getTimeInterval();
    }

    /**
     * Sets time interval between triad samples expressed in
     * seconds (s).
     *
     * @param timeInterval time interval between triad samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if estimator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        windowedNoiseEstimator.setTimeInterval(timeInterval);
        accumulatedNoiseEstimator.setTimeInterval(timeInterval);
    }

    /**
     * Gets time interval between triad samples.
     *
     * @return time interval between triad samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(getTimeInterval(), TimeUnit.SECOND);
    }

    /**
     * Gets time interval between triad samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(getTimeInterval());
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between triad samples.
     *
     * @param timeInterval time interval between triad samples.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(TimeConverter.convert(timeInterval.getValue().doubleValue(), timeInterval.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Gets current status of this detector.
     *
     * @return current status of this detector.
     */
    public Status getStatus() {
        return status;
    }

    /**
     * Gets measurement base noise level that has been detected during
     * initialization expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or
     * Teslas (T) for magnetic flux density.
     *
     * @return base noise level.
     */
    public double getBaseNoiseLevel() {
        return baseNoiseLevel;
    }

    /**
     * Gets measurement base noise level that has been detected during
     * initialization.
     *
     * @return measurement base noise level.
     */
    public M getBaseNoiseLevelAsMeasurement() {
        return createMeasurement(baseNoiseLevel, getDefaultUnit());
    }

    /**
     * Gets measurement base noise level that has been detected during
     * initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getBaseNoiseLevelAsMeasurement(final M result) {
        result.setValue(baseNoiseLevel);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in
     * (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for
     * magnetometer.
     *
     * @return measurement base noise level PSD.
     */
    public double getBaseNoiseLevelPsd() {
        return baseNoiseLevel * baseNoiseLevel * getTimeInterval();
    }

    /**
     * Gets measurement base noise level root PSD (Power SpectralDensity) expressed
     * in (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5)
     * for magnetometer.
     *
     * @return measurement base noise level root PSD.
     */
    public double getBaseNoiseLevelRootPsd() {
        return baseNoiseLevel * Math.sqrt(getTimeInterval());
    }

    /**
     * Gets threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2) for acceleration, radians per second
     * (rad/s) for angular speed or Teslas (T) for magnetic flux density.
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @return threshold to determine static/dynamic period changes.
     */
    public M getThresholdAsMeasurement() {
        return createMeasurement(threshold, getDefaultUnit());
    }

    /**
     * Gets threshold to determine static/dynamic period changes.
     *
     * @param result instance where result will be stored.
     */
    public void getThresholdAsMeasurement(final M result) {
        result.setValue(threshold);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Indicates whether this detector is busy processing last provided sample.
     *
     * @return true if this detector is busy, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getProcessedSamples() {
        return processedSamples;
    }

    /**
     * Gets average x-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average x-coordinate of measurement during last static
     * period.
     */
    public double getAccumulatedAvgX() {
        return accumulatedAvgX;
    }

    /**
     * Gets average x-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average x-coordinate of measurement during last static
     * period.
     */
    public M getAccumulatedAvgXAsMeasurement() {
        return createMeasurement(accumulatedAvgX, getDefaultUnit());
    }

    /**
     * Gets average x-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedAvgXAsMeasurement(final M result) {
        result.setValue(accumulatedAvgX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets average y-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average y-coordinate of measurement during last static
     * period.
     */
    public double getAccumulatedAvgY() {
        return accumulatedAvgY;
    }

    /**
     * Gets average y-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average y-coordinate of measurement during last static
     * period.
     */
    public M getAccumulatedAvgYAsMeasurement() {
        return createMeasurement(accumulatedAvgY, getDefaultUnit());
    }

    /**
     * Gets average y-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedAvgYAsMeasurement(final M result) {
        result.setValue(accumulatedAvgY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets average z-coordinate of measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average y-coordinate of measurement during last static
     * period.
     */
    public double getAccumulatedAvgZ() {
        return accumulatedAvgZ;
    }

    /**
     * Gets average z-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average z-coordinate of measurement during last static
     * period.
     */
    public M getAccumulatedAvgZAsMeasurement() {
        return createMeasurement(accumulatedAvgZ, getDefaultUnit());
    }

    /**
     * Gets average z-coordinate of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedAvgZAsMeasurement(final M result) {
        result.setValue(accumulatedAvgZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets average measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return accumulated average measurements triad during last static period.
     */
    public T getAccumulatedAvgTriad() {
        return createTriad(accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ, getDefaultUnit());
    }

    /**
     * Gets average measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedAvgTriad(final T result) {
        result.setValueCoordinatesAndUnit(accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ, getDefaultUnit());
    }

    /**
     * Gets standard deviation of x-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of x-coordinate of measurements accumulated
     * during last static period.
     */
    public double getAccumulatedStdX() {
        return accumulatedStdX;
    }

    /**
     * Gets standard deviation of x-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of x-coordinate of measurements accumulated
     * during last static period.
     */
    public M getAccumulatedStdXAsMeasurement() {
        return createMeasurement(accumulatedStdX, getDefaultUnit());
    }

    /**
     * Gets standard deviation of x-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedStdXAsMeasurement(final M result) {
        result.setValue(accumulatedStdX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets standard deviation of y-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of y-coordinate of measurements accumulated
     * during last static period.
     */
    public double getAccumulatedStdY() {
        return accumulatedStdY;
    }

    /**
     * Gets standard deviation of y-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of y-coordinate of measurements accumulated
     * during last static period.
     */
    public M getAccumulatedStdYAsMeasurement() {
        return createMeasurement(accumulatedStdY, getDefaultUnit());
    }

    /**
     * Gets standard deviation of y-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedStdYAsMeasurement(final M result) {
        result.setValue(accumulatedStdY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets standard deviation of z-coordinate of measurements accumulated during
     * last static period expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of z-coordinate of measurements accumulated
     * during last static period.
     */
    public double getAccumulatedStdZ() {
        return accumulatedStdZ;
    }

    /**
     * Gets standard deviation of z-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of z-coordinate of measurements accumulated
     * during last static period.
     */
    public M getAccumulatedStdZAsMeasurement() {
        return createMeasurement(accumulatedStdZ, getDefaultUnit());
    }

    /**
     * Gets standard deviation of z-coordinate of measurements accumulated during
     * last static period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedStdZAsMeasurement(final M result) {
        result.setValue(accumulatedStdZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets standard deviation of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @return standard deviation of measurements accumulated during last static
     * period.
     */
    public T getAccumulatedStdTriad() {
        return createTriad(accumulatedStdX, accumulatedStdY, accumulatedStdZ, getDefaultUnit());
    }

    /**
     * Gets standard deviation of measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic
     * one or after completing initialization.
     *
     * @param result instance where result will be stored.
     */
    public void getAccumulatedStdTriad(final T result) {
        result.setValueCoordinatesAndUnit(accumulatedStdX, accumulatedStdY, accumulatedStdZ, getDefaultUnit());
    }

    /**
     * Gets windowed average x-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average x-coordinate of measurements for each processed
     * triad.
     */
    public double getInstantaneousAvgX() {
        return instantaneousAvgX;
    }

    /**
     * Gets windowed average x-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average x-coordinate of measurements for each processed
     * triad.
     */
    public M getInstantaneousAvgXAsMeasurement() {
        return createMeasurement(instantaneousAvgX, getDefaultUnit());
    }

    /**
     * Gets windowed average x-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousAvgXAsMeasurement(final M result) {
        result.setValue(instantaneousAvgX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed average y-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average y-coordinate of measurements for each processed
     * triad.
     */
    public double getInstantaneousAvgY() {
        return instantaneousAvgY;
    }

    /**
     * Gets windowed average y-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average y-coordinate of measurements for each processed
     * triad.
     */
    public M getInstantaneousAvgYAsMeasurement() {
        return createMeasurement(instantaneousAvgY, getDefaultUnit());
    }

    /**
     * Gets windowed average y-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousAvgYAsMeasurement(final M result) {
        result.setValue(instantaneousAvgY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed average z-coordinate of measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for
     * magnetic flux density.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average z-coordinate of measurements for each processed
     * triad.
     */
    public double getInstantaneousAvgZ() {
        return instantaneousAvgZ;
    }

    /**
     * Gets windowed average z-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average z-coordinate of measurements for each processed
     * triad.
     */
    public M getInstantaneousAvgZAsMeasurement() {
        return createMeasurement(instantaneousAvgZ, getDefaultUnit());
    }

    /**
     * Gets windowed average z-coordinate of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousAvgZAsMeasurement(final M result) {
        result.setValue(instantaneousAvgZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed average of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @return windowed average of measurements for each processed triad.
     */
    public T getInstantaneousAvgTriad() {
        return createTriad(instantaneousAvgX, instantaneousAvgY, instantaneousAvgZ, getDefaultUnit());
    }

    /**
     * Gets windowed average of measurements for each processed triad.
     * This value is updated for each processed sample containing an average
     * value for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousAvgTriad(final T result) {
        result.setValueCoordinatesAndUnit(instantaneousAvgX, instantaneousAvgY, instantaneousAvgZ, getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of x-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of x-coordinate of measurements for
     * each processed triad.
     */
    public double getInstantaneousStdX() {
        return instantaneousStdX;
    }

    /**
     * Gets windowed standard deviation of x-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of x-coordinate of measurements for
     * each processed triad.
     */
    public M getInstantaneousStdXAsMeasurement() {
        return createMeasurement(instantaneousStdX, getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of x-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousStdXAsMeasurement(final M result) {
        result.setValue(instantaneousStdX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of y-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas
     * (T) for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of y-coordinate of measurements for
     * each processed triad.
     */
    public double getInstantaneousStdY() {
        return instantaneousStdY;
    }

    /**
     * Gets windowed standard deviation of y-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation of the samples within the window.
     *
     * @return windowed standard deviation of y-coordinate of measurements for
     * each processed triad.
     */
    public M getInstantaneousStdYAsMeasurement() {
        return createMeasurement(instantaneousStdY, getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of y-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation of the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousStdYAsMeasurement(final M result) {
        result.setValue(instantaneousStdY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of z-coordinate of measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for
     * acceleration, radians per second (rad/s) for angular speed or Teslas (T)
     * for magnetic flux density.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of z-coordinate of measurements for
     * each processed triad.
     */
    public double getInstantaneousStdZ() {
        return instantaneousStdZ;
    }

    /**
     * Gets windowed standard deviation of z-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of z-coordinate of measurements for
     * each processed triad.
     */
    public M getInstantaneousStdZAsMeasurement() {
        return createMeasurement(instantaneousStdZ, getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of z-coordinate of measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousStdZAsMeasurement(final M result) {
        result.setValue(instantaneousStdZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @return windowed standard deviation of measurements for each processed
     * triad.
     */
    public T getInstantaneousStdTriad() {
        return createTriad(instantaneousStdX, instantaneousStdY, instantaneousStdZ, getDefaultUnit());
    }

    /**
     * Gets windowed standard deviation of measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard
     * deviation for the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    public void getInstantaneousStdTriad(final T result) {
        result.setValueCoordinatesAndUnit(instantaneousStdX, instantaneousStdY, instantaneousStdZ, getDefaultUnit());
    }

    /**
     * Processes a new measurement triad sample.
     *
     * @param triad a new measurement triad to be processed.
     * @return true if provided triad has been processed, false if provided triad has been skipped because detector
     * previously failed. If detector previously failed, it will need to be reset before processing additional
     * samples.
     * @throws LockedException if detector is busy processing a previous sample.
     */
    public boolean process(final T triad) throws LockedException {
        return process(convertMeasurement(triad.getValueX(), triad.getUnit()),
                convertMeasurement(triad.getValueY(), triad.getUnit()),
                convertMeasurement(triad.getValueZ(), triad.getUnit()));
    }

    /**
     * Processes a new measurement triad sample.
     *
     * @param valueX x-coordinate of sensed measurement.
     * @param valueY y-coordinate of sensed measurement.
     * @param valueZ z-coordinate of sensed measurement.
     * @return true if provided triad has been processed, false if provided triad has been skipped because detector
     * previously failed. If detector previously failed, it will need to be reset before processing additional
     * samples.
     * @throws LockedException if detector is busy processing a previous sample.
     */
    public boolean process(final M valueX, final M valueY, final M valueZ) throws LockedException {
        return process(convertMeasurement(valueX), convertMeasurement(valueY), convertMeasurement(valueZ));
    }

    /**
     * Processes a new measurement triad sample.
     * Provided measurement coordinates are expressed in meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
     *
     * @param valueX x-coordinate of sensed measurement.
     * @param valueY y-coordinate of sensed measurement.
     * @param valueZ z-coordinate of sensed measurement.
     * @return true if provided triad has been processed, false if provided triad has been skipped because detector
     * previously failed. If detector previously failed, it will need to be reset before processing additional
     * samples.
     * @throws LockedException if detector is busy processing a previous sample.
     */
    public boolean process(final double valueX, final double valueY, final double valueZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (status == Status.FAILED) {
            return false;
        }

        running = true;

        if (status == Status.IDLE) {
            // start initialization
            status = Status.INITIALIZING;

            if (listener != null) {
                //noinspection unchecked
                listener.onInitializationStarted((D) this);
            }
        }

        processedSamples++;

        windowedNoiseEstimator.addTriadAndProcess(valueX, valueY, valueZ);

        instantaneousAvgX = windowedNoiseEstimator.getAvgX();
        instantaneousAvgY = windowedNoiseEstimator.getAvgY();
        instantaneousAvgZ = windowedNoiseEstimator.getAvgZ();

        instantaneousStdX = windowedNoiseEstimator.getStandardDeviationX();
        instantaneousStdY = windowedNoiseEstimator.getStandardDeviationY();
        instantaneousStdZ = windowedNoiseEstimator.getStandardDeviationZ();

        final var windowedStdNorm = windowedNoiseEstimator.getStandardDeviationNorm();

        final var filledWindow = windowedNoiseEstimator.isWindowFilled();

        if (status == Status.INITIALIZING) {
            // process sample during initialization
            accumulatedNoiseEstimator.addTriad(valueX, valueY, valueZ);
            final var accumulatedStdNorm = accumulatedNoiseEstimator.getStandardDeviationNorm();

            if (processedSamples < initialStaticSamples) {
                if (filledWindow && (windowedStdNorm / accumulatedStdNorm > instantaneousNoiseLevelFactor)) {
                    // sudden motion detected
                    status = Status.FAILED;

                    // notify error
                    if (listener != null) {
                        //noinspection unchecked
                        listener.onError((D) this, accumulatedStdNorm, windowedStdNorm,
                                ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED);
                    }
                }

            } else if (filledWindow) {
                // initialization completed
                // set base noise level and threshold
                baseNoiseLevel = accumulatedStdNorm;
                threshold = baseNoiseLevel * thresholdFactor;

                // keep average/std measurements triad in case we want to obtain
                // its value since initial period must be static
                accumulatedAvgX = accumulatedNoiseEstimator.getAvgX();
                accumulatedAvgY = accumulatedNoiseEstimator.getAvgY();
                accumulatedAvgZ = accumulatedNoiseEstimator.getAvgZ();

                accumulatedStdX = accumulatedNoiseEstimator.getStandardDeviationX();
                accumulatedStdY = accumulatedNoiseEstimator.getStandardDeviationY();
                accumulatedStdZ = accumulatedNoiseEstimator.getStandardDeviationZ();

                // reset accumulated estimator so that we can estimate
                // average specific force in static periods
                accumulatedNoiseEstimator.reset();

                if (baseNoiseLevel > baseNoiseLevelAbsoluteThreshold) {
                    // base noise level exceeds allowed value
                    status = Status.FAILED;

                    // notify error
                    if (listener != null) {
                        //noinspection unchecked
                        listener.onError((D) this, accumulatedStdNorm, windowedStdNorm,
                                ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED);
                    }

                } else {
                    // initialization has been successfully completed
                    status = Status.INITIALIZATION_COMPLETED;

                    if (listener != null) {
                        //noinspection unchecked
                        listener.onInitializationCompleted((D) this, baseNoiseLevel);
                    }
                }

            }

            running = false;
            return true;
        } else {
            // detect static or dynamic period
            final var previousStatus = status;

            if (windowedStdNorm < threshold) {
                status = Status.STATIC_INTERVAL;
            } else {
                status = Status.DYNAMIC_INTERVAL;
            }

            if (status == Status.STATIC_INTERVAL) {
                // while we are in static interval, keep adding samples to estimate
                // accumulated average measurement triad
                accumulatedNoiseEstimator.addTriad(valueX, valueY, valueZ);
            }

            if (previousStatus != status) {
                // static/dynamic period change detected
                if (status == Status.STATIC_INTERVAL && listener != null) {
                    //noinspection unchecked
                    listener.onStaticIntervalDetected((D) this,
                            instantaneousAvgX, instantaneousAvgY, instantaneousAvgZ,
                            instantaneousStdX, instantaneousStdY, instantaneousStdZ);
                } else if (status == Status.DYNAMIC_INTERVAL) {
                    // when switching from static to dynamic interval,
                    // pick accumulated average and standard deviation measurement triads
                    accumulatedAvgX = accumulatedNoiseEstimator.getAvgX();
                    accumulatedAvgY = accumulatedNoiseEstimator.getAvgY();
                    accumulatedAvgZ = accumulatedNoiseEstimator.getAvgZ();

                    accumulatedStdX = accumulatedNoiseEstimator.getStandardDeviationX();
                    accumulatedStdY = accumulatedNoiseEstimator.getStandardDeviationY();
                    accumulatedStdZ = accumulatedNoiseEstimator.getStandardDeviationZ();

                    // reset accumulated estimator when switching to dynamic period
                    accumulatedNoiseEstimator.reset();

                    if (listener != null) {
                        //noinspection unchecked
                        listener.onDynamicIntervalDetected((D) this,
                                instantaneousAvgX, instantaneousAvgY, instantaneousAvgZ,
                                instantaneousStdX, instantaneousStdY, instantaneousStdZ,
                                accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ,
                                accumulatedStdX, accumulatedStdY, accumulatedStdZ);
                    }
                }
            }
        }

        running = false;
        return true;
    }

    /**
     * Resets this detector so that it is initialized again when new samples are added.
     *
     * @throws LockedException if detector is busy.
     */
    public void reset() throws LockedException {
        if (running) {
            throw new LockedException();
        }

        running = true;

        status = Status.IDLE;
        processedSamples = 0;
        baseNoiseLevel = 0.0;
        threshold = 0.0;

        windowedNoiseEstimator.reset();
        accumulatedNoiseEstimator.reset();

        if (listener != null) {
            //noinspection unchecked
            listener.onReset((D) this);
        }

        running = false;
    }

    /**
     * Converts provided measurement instance to its default unit, which is
     * meters per squared second (m/s^2) for acceleration, radians per second (rad/s) for
     * angular speed or Teslas (T) for magnetic flux density.
     *
     * @param measurement measurement to be converted.
     * @return converted value.
     */
    protected double convertMeasurement(M measurement) {
        return convertMeasurement(measurement.getValue().doubleValue(), measurement.getUnit());
    }

    /**
     * Converts provided measurement value expressed in provided unit to the
     * default measurement value, which is meters per squared second (m/s^2) for acceleration,
     * radians per second (rad/s) for angular speed or Teslas (t) for magnetic flux density.
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted.
     * @return converted value.
     */
    protected abstract double convertMeasurement(final double value, final U unit);

    /**
     * Creates a measurement instance using provided value and unit.
     *
     * @param value value of measurement.
     * @param unit  unit of value.
     * @return created measurement
     */
    protected abstract M createMeasurement(final double value, final U unit);

    /**
     * Gets default unit for measurements this implementation works with.
     *
     * @return default measurement unit.
     */
    protected abstract U getDefaultUnit();

    /**
     * Creates a triad.
     *
     * @param valueX x-coordinate value.
     * @param valueY y-coordinate value.
     * @param valueZ z-coordinate value.
     * @param unit   unit of values.
     * @return created triad.
     */
    protected abstract T createTriad(final double valueX, final double valueY, final double valueZ, final U unit);

    /**
     * Possible detector status values.
     */
    public enum Status {
        /**
         * Detector is in idle status when it hasn't processed any sample yet.
         */
        IDLE,

        /**
         * Detector is processing samples in the initial static process to determine base noise level.
         */
        INITIALIZING,

        /**
         * Detector has successfully completed processing samples on the initial
         * static period.
         */
        INITIALIZATION_COMPLETED,

        /**
         * A static interval has been detected, where accelerometer is considered to be subject to no substantial
         * movement forces.
         */
        STATIC_INTERVAL,

        /**
         * A dynamic interval has been detected, where accelerometer is considered to be subject to substantial
         * movement forces.
         */
        DYNAMIC_INTERVAL,

        /**
         * Detector has failed. This happens if accelerometer is subject to sudden movement forces while detector
         * is initializing during the initial static period.
         * When detector has failed, no new samples will be allowed to be processed until detector is reset.
         */
        FAILED
    }

    /**
     * Reason why this detector has failed during initialization.
     */
    public enum ErrorReason {
        /**
         * If a sudden movement is detected during initialization.
         */
        SUDDEN_EXCESSIVE_MOVEMENT_DETECTED,

        /**
         * If overall noise level is excessive during initialization.
         */
        OVERALL_EXCESSIVE_MOVEMENT_DETECTED
    }
}
