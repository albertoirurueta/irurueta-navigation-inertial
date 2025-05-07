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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator;
import com.irurueta.units.Measurement;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Base class to estimate measurement noise variances and PSD's (Power Spectral Densities)
 * along with their average values.
 * Implementations of this estimator may use norms of measurement triads to estimate noise
 * levels.
 * To compute PSD's, this estimator assumes that measurement samples are obtained
 * at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, accelerometer sampling rate average can be estimated using
 * {@link TimeIntervalEstimator}.
 *
 * @param <U> a measurement unit type.
 * @param <M> a measurement type.
 * @param <E> an estimator type.
 * @param <L> a listener type.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class AccumulatedMeasurementNoiseEstimator<U extends Enum<?>,
        M extends Measurement<U>,
        E extends AccumulatedMeasurementNoiseEstimator<U, M, E, L>,
        L extends AccumulatedMeasurementNoiseEstimatorListener<U, M, E>> {

    /**
     * Default time interval between accelerometer samples expressed in seconds
     * (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Time interval expressed in seconds (s) between consecutive accelerometer
     * samples.
     */
    private double timeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Listener to handle events raised by this estimator.
     */
    private L listener;

    /**
     * Last provided measurement.
     */
    private M lastMeasurement;

    /**
     * Contains estimated average of measurement expressed in its default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     */
    private double avg;

    /**
     * Contains estimated variance of measurement expressed in its default squared unit
     * (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for magnetic
     * flux density).
     */
    private double variance;

    /**
     * Number of processed body kinematics samples.
     */
    private int numberOfProcessedSamples;

    /**
     * Number of processed timestamp samples plus one.
     */
    private int numberOfProcessedSamplesPlusOne = 1;

    /**
     * Indicates that estimator is running.
     */
    private boolean running;

    /**
     * Constructor.
     */
    protected AccumulatedMeasurementNoiseEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    protected AccumulatedMeasurementNoiseEstimator(final L listener) {
        this.listener = listener;
    }

    /**
     * Gets time interval between accelerometer triad samples expressed in
     * seconds (s).
     *
     * @return time interval between accelerometer triad samples.
     */
    public double getTimeInterval() {
        return timeInterval;
    }

    /**
     * Sets time interval between accelerometer triad samples expressed in
     * seconds (s).
     *
     * @param timeInterval time interval between accelerometer triad samples.
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

        this.timeInterval = timeInterval;
    }

    /**
     * Gets time interval between accelerometer triad samples.
     *
     * @return time interval between accelerometer triad samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(timeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between accelerometer triad samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(timeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between accelerometer triad samples.
     *
     * @param timeInterval time interval between accelerometer triad samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public L getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final L listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets last provided measurement or null if not available.
     *
     * @return last provided measurement or null.
     */
    public M getLastMeasurement() {
        return lastMeasurement;
    }

    /**
     * Gets last provided measurement.
     *
     * @param result instance where last provided measurement will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getLastMeasurement(final M result) {
        if (lastMeasurement != null) {
            result.setValue(lastMeasurement.getValue());
            result.setUnit(lastMeasurement.getUnit());
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated average of measurement expressed in its default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     *
     * @return average of measurement in current window.
     */
    public double getAvg() {
        return avg;
    }

    /**
     * Gets estimated average of measurement within current window.
     *
     * @return average of measurement in current window
     */
    public M getAvgAsMeasurement() {
        return createMeasurement(avg, getDefaultUnit());
    }

    /**
     * Gets estimated average of measurement within current window.
     *
     * @param result instance where average of measurement will be stored.
     */
    public void getAvgAsMeasurement(final M result) {
        result.setValue(avg);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated variance of measurement within current window
     * expressed in its default squared unit (m^2/s^4 for acceleration,
     * rad^2/s^2 for angular speed or T^2 for magnetic flux density).
     *
     * @return estimated variance of measurement within current window.
     */
    public double getVariance() {
        return variance;
    }

    /**
     * Gets estimated standard deviation of measurement within current window
     * and expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @return estimated standard of measurement.
     */
    public double getStandardDeviation() {
        return Math.sqrt(variance);
    }

    /**
     * Gets estimated standard deviation of measurement within current window.
     *
     * @return estimated standard deviation of measurement.
     */
    public M getStandardDeviationAsMeasurement() {
        return createMeasurement(getStandardDeviation(), getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of measurement within current window.
     *
     * @param result instance where estimated standard deviation of measurement
     *               will be stored.
     */
    public void getStandardDeviationAsMeasurement(final M result) {
        result.setValue(getStandardDeviation());
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets measurement noise PSD (Power Spectral Density) expressed
     * in (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for
     * magnetometer.
     *
     * @return measurement noise PSD.
     */
    public double getPsd() {
        return variance * timeInterval;
    }

    /**
     * Gets measurement noise root PSD (Power Spectral Density) expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for
     * magnetometer.
     *
     * @return measurement noise root PSD.
     */
    public double getRootPsd() {
        return Math.sqrt(getPsd());
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getNumberOfProcessedSamples() {
        return numberOfProcessedSamples;
    }

    /**
     * Indicates whether estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Adds a measurement value expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @param value value to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addMeasurement(final double value) throws LockedException {

        if (running) {
            throw new LockedException();
        }

        running = true;

        if (lastMeasurement == null && listener != null) {
            //noinspection unchecked
            listener.onStart((E) this);
        }

        // compute average
        final var tmp = (double) numberOfProcessedSamples / (double) numberOfProcessedSamplesPlusOne;
        avg = avg * tmp + value / numberOfProcessedSamplesPlusOne;

        // compute variance
        final var diff = value - avg;
        final var diff2 = diff * diff;

        variance = variance * tmp + diff2 / numberOfProcessedSamplesPlusOne;

        if (lastMeasurement == null) {
            lastMeasurement = createMeasurement(value, getDefaultUnit());
        } else {
            lastMeasurement.setValue(value);
            lastMeasurement.setUnit(getDefaultUnit());
        }

        numberOfProcessedSamples++;
        numberOfProcessedSamplesPlusOne++;

        if (listener != null) {
            //noinspection unchecked
            listener.onMeasurementAdded((E) this);
        }

        running = false;
    }

    /**
     * Adds a measurement value.
     *
     * @param measurement measurement to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addMeasurement(final M measurement) throws LockedException {
        addMeasurement(convertToDefaultUnit(measurement));
    }

    /**
     * Resets current estimator.
     *
     * @return true if estimator was successfully reset, false if no reset was needed.
     * @throws LockedException if estimator is currently running.
     */
    public boolean reset() throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (numberOfProcessedSamples == 0) {
            return false;
        }

        running = true;
        lastMeasurement = null;
        avg = 0.0;
        variance = 0.0;
        numberOfProcessedSamples = 0;
        numberOfProcessedSamplesPlusOne = 1;

        if (listener != null) {
            //noinspection unchecked
            listener.onReset((E) this);
        }

        running = false;

        return true;
    }

    /**
     * Gets default unit for a measurement.
     *
     * @return default unit for a measurement.
     */
    protected abstract U getDefaultUnit();

    /**
     * Creates a measurement with provided value and unit.
     *
     * @param value value to be set.
     * @param unit  unit to be set.
     * @return created measurement.
     */
    protected abstract M createMeasurement(final double value, final U unit);

    /**
     * Converts provided measurement into default unit.
     *
     * @param value measurement to be converted.
     * @return converted value.
     */
    protected abstract double convertToDefaultUnit(M value);
}
