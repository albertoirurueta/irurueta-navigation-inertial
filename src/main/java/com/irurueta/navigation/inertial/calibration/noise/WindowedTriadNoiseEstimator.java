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
import com.irurueta.navigation.inertial.calibration.Triad;
import com.irurueta.units.Measurement;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.LinkedList;

/**
 * Base class to estimate measurement noise variances and PSD's (Power Spectral Densities)
 * along with average values for a windowed amount of samples.
 * Implementations of this estimator must be used when the body where the measurement device
 * is attached to remains static on the same position with zero velocity while capturing data.
 * To compute PSD's, this estimator assumes that measurement samples are obtained
 * at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, accelerometer sampling rate average can be estimated using
 * {@link TimeIntervalEstimator}.
 * This estimator does NOT require the knowledge of current location and body
 * orientation.
 * Because body location and orientation is not known, estimated average values
 * cannot be used to determine biases. Only norm of noise estimations
 * (variance or standard deviation) can be safely used.
 * Notice that if there are less than {@link #getWindowSize()} processed
 * samples in the window, this estimator will assume that the remaining ones
 * until the window is completed have zero values.
 *
 * @param <U> a measurement unit type.
 * @param <M> a measurement type.
 * @param <T> a triad type.
 * @param <E> an estimator type.
 * @param <L> a listener type.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class WindowedTriadNoiseEstimator<U extends Enum<?>,
        M extends Measurement<U>, T extends Triad<U, M>,
        E extends WindowedTriadNoiseEstimator<U, M, T, E, L>,
        L extends WindowedTriadNoiseEstimatorListener<U, M, T, E>> {

    /**
     * Number of samples to keep within the window by default.
     * For an accelerometer generating 100 samples/second, this is equivalent to
     * 1 second.
     * For an accelerometer generating 50 samples/second, this is equivalent to
     * 2 seconds.
     */
    public static final int DEFAULT_WINDOW_SIZE = 101;

    /**
     * Minimum allowed window size.
     */
    public static final int MIN_WINDOW_SIZE = 3;

    /**
     * Default time interval between accelerometer samples expressed in seconds
     * (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value and must have
     * an odd value.
     */
    private int windowSize = DEFAULT_WINDOW_SIZE;

    /**
     * Time interval expressed in seconds (s) between consecutive triad
     * samples.
     */
    private double timeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Keeps the list of triad samples that remain within the window.
     */
    private final LinkedList<T> windowedSamples = new LinkedList<>();

    /**
     * Listener to handle events raised by this estimator.
     */
    private L listener;

    /**
     * Contains estimated average of x coordinate of measurement expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     */
    private double avgX;

    /**
     * Contains estimated average of y coordinate of measurement expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     */
    private double avgY;

    /**
     * Contains estimated average of z coordinate of measurement expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     */
    private double avgZ;

    /**
     * Contains estimated variance of x coordinate of measurement expressed in its default
     * squared unit (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for magnetic
     * flux density).
     */
    private double varianceX;

    /**
     * Contains estimated variance of y coordinate of measurement expressed in its default
     * squared unit (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for magnetic
     * flux density).
     */
    private double varianceY;

    /**
     * Contains estimated variance of x coordinate of measurement expressed in its default
     * squared unit (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for magnetic
     * flux density).
     */
    private double varianceZ;

    /**
     * Number of processed acceleration triad samples.
     */
    private int numberOfProcessedSamples;

    /**
     * Indicates whether estimator is running or not.
     */
    private boolean running;

    /**
     * Constructor.
     */
    protected WindowedTriadNoiseEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    protected WindowedTriadNoiseEstimator(final L listener) {
        this.listener = listener;
    }

    /**
     * Gets length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value and must
     * have an odd value.
     *
     * @return length of number of samples to keep within the window.
     */
    public int getWindowSize() {
        return windowSize;
    }

    /**
     * Sets length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value and must have
     * an odd value.
     * When window size is modified, instance state is reset.
     *
     * @param windowSize length of number of samples to keep within the window.
     * @throws IllegalArgumentException if provided value is not valid.
     * @throws LockedException          if estimator is currently running.
     */
    public void setWindowSize(final int windowSize) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        // check that window is larger than minimum allowed value
        if (windowSize < MIN_WINDOW_SIZE) {
            throw new IllegalArgumentException();
        }

        // check that window size is not even
        if (windowSize % 2 == 0) {
            throw new IllegalArgumentException();
        }

        this.windowSize = windowSize;
        reset();
    }

    /**
     * Gets time interval between triad samples expressed in
     * seconds (s).
     *
     * @return time interval between triad samples.
     */
    public double getTimeInterval() {
        return timeInterval;
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

        this.timeInterval = timeInterval;
    }

    /**
     * Gets time interval between triad samples.
     *
     * @return time interval between triad samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(timeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between triad samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(timeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between triad samples.
     *
     * @param timeInterval time interval between triad samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(TimeConverter.convert(timeInterval.getValue().doubleValue(), timeInterval.getUnit(),
                TimeUnit.SECOND));
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
     * Gets first provided measurement triad within the window.
     *
     * @return first provided measurement triad within the window or null if not
     * available.
     */
    public T getFirstWindowedTriad() {
        return windowedSamples.isEmpty() ? null : windowedSamples.getFirst();
    }

    /**
     * Gets first provided measurement triad within the window.
     *
     * @param result instance where first provided measurement triad will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getFirstWindowedTriad(final T result) {
        if (windowedSamples.isEmpty()) {
            return false;
        } else {
            result.copyFrom(windowedSamples.getFirst());
            return true;
        }
    }

    /**
     * Gets last provided measurement triad within the window.
     *
     * @return last provided measurement triad within the window or null if not
     * available.
     */
    public T getLastWindowedTriad() {
        return windowedSamples.isEmpty() ? null : windowedSamples.getLast();
    }

    /**
     * Gets last provided measurement triad within the window.
     *
     * @param result instance where last provided measurement triad will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getLastWindowedTriad(final T result) {
        if (windowedSamples.isEmpty()) {
            return false;
        } else {
            result.copyFrom(windowedSamples.getLast());
            return true;
        }
    }

    /**
     * Gets estimated average of x coordinate of measurement expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of measurement in current window.
     */
    public double getAvgX() {
        return avgX;
    }

    /**
     * Gets estimated average of x coordinate of measurement within current window.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of measurement in current window.
     */
    public M getAvgXAsMeasurement() {
        return createMeasurement(avgX, getDefaultUnit());
    }

    /**
     * Gets estimated average of x coordinate of measurement within current window.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of x coordinate of measurement will be stored.
     */
    public void getAvgXAsMeasurement(final M result) {
        result.setValue(avgX);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated average of y coordinate of measurement expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of measurement in current window.
     */
    public double getAvgY() {
        return avgY;
    }

    /**
     * Gets estimated average of y coordinate of measurement within current window.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of measurement in current window.
     */
    public M getAvgYAsMeasurement() {
        return createMeasurement(avgY, getDefaultUnit());
    }

    /**
     * Gets estimated average of y coordinate of measurement within current window.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of y coordinate of measurement will be stored.
     */
    public void getAvgYAsMeasurement(final M result) {
        result.setValue(avgY);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated average of z coordinate of measurement expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of measurement in current window.
     */
    public double getAvgZ() {
        return avgZ;
    }

    /**
     * Gets estimated average of z coordinate of measurement within current window.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of measurement in current window.
     */
    public M getAvgZAsMeasurement() {
        return createMeasurement(avgZ, getDefaultUnit());
    }

    /**
     * Gets estimated average of z coordinate of measurement within current window.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of z coordinate of measurement will be stored.
     */
    public void getAvgZAsMeasurement(final M result) {
        result.setValue(avgZ);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated average as a measurement triad.
     *
     * @return average measurement triad.
     */
    public T getAvgTriad() {
        return createTriad(avgX, avgY, avgZ, getDefaultUnit());
    }

    /**
     * Gets estimated average as a measurement triad.
     *
     * @param result instance where average values and unit will be stored.
     */
    public void getAvgTriad(final T result) {
        result.setValueCoordinatesAndUnit(avgX, avgY, avgZ, getDefaultUnit());
    }

    /**
     * Gets norm of estimated average measurement within current window expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This value is independent of body orientation.
     *
     * @return norm of estimated average specific force.
     */
    public double getAvgNorm() {
        return Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
    }

    /**
     * Gets norm of estimated average measurement within current window.
     *
     * @return norm of estimated average measurement.
     */
    public M getAvgNormAsMeasurement() {
        return createMeasurement(getAvgNorm(), getDefaultUnit());
    }

    /**
     * Gets norm of estimated average measurement within current window.
     *
     * @param result instance where norm of estimated average measurement will be stored.
     */
    public void getAvgNormAsMeasurement(final M result) {
        result.setValue(getAvgNorm());
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated variance of x coordinate of measurement within current window
     * expressed in its default squared unit (m^2/s^4 for acceleration,
     * rad^2/s^2 for angular speed or T^2 for magnetic flux density).
     *
     * @return estimated variance of x coordinate of measurement within current
     * window.
     */
    public double getVarianceX() {
        return varianceX;
    }

    /**
     * Gets estimated variance of y coordinate of measurement within current window
     * expressed in its default squared unit (m^2/s^4 for acceleration,
     * rad^2/s^2 for angular speed or T^2 for magnetic flux density).
     *
     * @return estimated variance of y coordinate of measurement within current
     * window.
     */
    public double getVarianceY() {
        return varianceY;
    }

    /**
     * Gets estimated variance of z coordinate of measurement within current window
     * expressed in its default squared unit (m^2/s^4 for acceleration,
     * rad^2/s^2 for angular speed or T^2 for magnetic flux density).
     *
     * @return estimated variance of z coordinate of measurement within current
     * window.
     */
    public double getVarianceZ() {
        return varianceZ;
    }

    /**
     * Gets estimated standard deviation of x coordinate of measurement within current
     * window and expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @return estimated standard deviation of x coordinate of measurement within
     * current window.
     */
    public double getStandardDeviationX() {
        return Math.sqrt(varianceX);
    }

    /**
     * Gets estimated standard deviation of x coordinate of measurement within current
     * window.
     *
     * @return estimated standard deviation of x coordinate of measurement.
     */
    public M getStandardDeviationXAsMeasurement() {
        return createMeasurement(getStandardDeviationX(), getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of x coordinate of measurement within current
     * window.
     *
     * @param result instance where estimated standard deviation of x coordinate of
     *               measurement will be stored.
     */
    public void getStandardDeviationXAsMeasurement(final M result) {
        result.setValue(getStandardDeviationX());
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of y coordinate of measurement within current
     * window and expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @return estimated standard deviation of y coordinate of measurement within
     * current window.
     */
    public double getStandardDeviationY() {
        return Math.sqrt(varianceY);
    }

    /**
     * Gets estimated standard deviation of y coordinate of measurement within current
     * window.
     *
     * @return estimated standard deviation of y coordinate of measurement.
     */
    public M getStandardDeviationYAsMeasurement() {
        return createMeasurement(getStandardDeviationY(), getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of y coordinate of measurement within current
     * window.
     *
     * @param result instance where estimated standard deviation of y coordinate of
     *               measurement will be stored.
     */
    public void getStandardDeviationYAsMeasurement(final M result) {
        result.setValue(getStandardDeviationY());
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of z coordinate of measurement within current
     * window and expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @return estimated standard deviation of z coordinate of measurement within
     * current window.
     */
    public double getStandardDeviationZ() {
        return Math.sqrt(varianceZ);
    }

    /**
     * Gets estimated standard deviation of z coordinate of measurement within current
     * window.
     *
     * @return estimated standard deviation of z coordinate of measurement.
     */
    public M getStandardDeviationZAsMeasurement() {
        return createMeasurement(getStandardDeviationZ(), getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of z coordinate of measurement within current
     * window.
     *
     * @param result instance where estimated standard deviation of z coordinate of
     *               measurement will be stored.
     */
    public void getStandardDeviationZAsMeasurement(final M result) {
        result.setValue(getStandardDeviationZ());
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of measurement within current window.
     *
     * @return estimated standard deviation triad of measurement.
     */
    public T getStandardDeviationTriad() {
        return createTriad(getStandardDeviationX(), getStandardDeviationY(), getStandardDeviationZ(), getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation of measurement within current window.
     *
     * @param result instance where estimated standard deviation triad of
     *               measurement will be stored.
     */
    public void getStandardDeviationTriad(final T result) {
        result.setValueCoordinatesAndUnit(getStandardDeviationX(), getStandardDeviationY(), getStandardDeviationZ(),
                getDefaultUnit());
    }

    /**
     * Gets norm of estimated standard deviation of measurement within current
     * window expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @return norm of estimated standard deviation of measurement.
     */
    public double getStandardDeviationNorm() {
        final var fx = getStandardDeviationX();
        final var fy = getStandardDeviationY();
        final var fz = getStandardDeviationZ();
        return Math.sqrt(fx * fx + fy * fy + fz * fz);
    }

    /**
     * Gets norm of estimated standard deviation of measurement within current window.
     *
     * @return norm of estimated standard deviation of measurement.
     */
    public M getStandardDeviationNormAsMeasurement() {
        return createMeasurement(getStandardDeviationNorm(), getDefaultUnit());
    }

    /**
     * Gets norm of estimated standard deviation of measurement within current window.
     *
     * @param result instance where norm of estimated standard deviation will be stored.
     */
    public void getStandardDeviationNormAsMeasurement(final M result) {
        result.setValue(getStandardDeviationNorm());
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets average of estimated standard deviation coordinates of measurement within
     * current window expressed in its default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @return average of estimated standard deviation coordinates.
     */
    public double getAverageStandardDeviation() {
        final var fx = getStandardDeviationX();
        final var fy = getStandardDeviationY();
        final var fz = getStandardDeviationZ();
        return (fx + fy + fz) / 3.0;
    }

    /**
     * Gets average of estimated standard deviation coordinates of measurement within
     * current window.
     *
     * @return average of estimated standard deviation coordinates.
     */
    public M getAverageStandardDeviationAsMeasurement() {
        return createMeasurement(getAverageStandardDeviation(), getDefaultUnit());
    }

    /**
     * Gets average of estimated standard deviation coordinates of measurement within
     * current window.
     *
     * @param result instance where average of estimated standard deviation coordinates
     *               will be stored.
     */
    public void getAverageStandardDeviationAsMeasurement(final M result) {
        result.setValue(getAverageStandardDeviation());
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets measurement noise PSD (Power Spectral Density) on x axis expressed
     * in (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for
     * magnetometer.
     *
     * @return measurement noise PSD on x axis.
     */
    public double getPsdX() {
        return varianceX * timeInterval;
    }

    /**
     * Gets measurement noise PSD (Power Spectral Density) on y axis expressed
     * in (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for
     * magnetometer.
     *
     * @return measurement noise PSD on y axis.
     */
    public double getPsdY() {
        return varianceY * timeInterval;
    }

    /**
     * Gets measurement noise PSD (Power Spectral Density) on z axis expressed
     * in (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for
     * magnetometer.
     *
     * @return measurement noise PSD on z axis.
     */
    public double getPsdZ() {
        return varianceZ * timeInterval;
    }

    /**
     * Gets measurement noise root PSD (Power Spectral Density) on x axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for
     * magnetometer.
     *
     * @return measurement noise root PSD on x axis.
     */
    public double getRootPsdX() {
        return Math.sqrt(getPsdX());
    }

    /**
     * Gets measurement noise root PSD (Power Spectral Density) on y axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for
     * magnetometer.
     *
     * @return measurement noise root PSD on y axis.
     */
    public double getRootPsdY() {
        return Math.sqrt(getPsdY());
    }

    /**
     * Gets measurement noise root PSD (Power Spectral Density) on z axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for
     * magnetometer.
     *
     * @return measurement noise root PSD on z axis.
     */
    public double getRootPsdZ() {
        return Math.sqrt(getPsdZ());
    }

    /**
     * Gets average measurement noise PSD (Power Spectral Density) among
     * x,y,z components expressed as (m^2 * s^-3) for accelerometer,
     * (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     *
     * @return average measurement noise PSD.
     */
    public double getAvgNoisePsd() {
        return (getPsdX() + getPsdY() + getPsdZ()) / 3.0;
    }

    /**
     * Gets norm of noise root PSD (Power Spectral Density) among x,y,z
     * components expressed as (m * s^-1.5) for accelerometer,
     * (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     *
     * @return norm of measurement noise root PSD.
     */
    public double getNoiseRootPsdNorm() {
        return Math.sqrt(getPsdX() + getPsdY() + getPsdZ());
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
     * Gets number of currently windowed samples.
     *
     * @return number of samples within the window.
     */
    public int getNumberOfSamplesInWindow() {
        return windowedSamples.size();
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
     * Indicates whether window of samples is filled or not.
     *
     * @return true if window is filled, false otherwise.
     */
    public boolean isWindowFilled() {
        return getNumberOfSamplesInWindow() == windowSize;
    }

    /**
     * Adds a triad of measurement samples and processes current window.
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param triad measurement triad to be added and processed.
     * @throws LockedException if estimator is currently running.
     */
    public void addTriadAndProcess(final T triad) throws LockedException {
        internalAdd(triad, true);
    }

    /**
     * Adds a triad of measurement samples and processes current window.
     * Values are expressed in measurement default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param valueX x coordinate of measurement to be added and processed.
     * @param valueY y coordinate of measurement to be added and processed.
     * @param valueZ z coordinate of measurement to be added and processed.
     * @throws LockedException if estimator is currently running.
     */
    public void addTriadAndProcess(final double valueX, final double valueY, final double valueZ)
            throws LockedException {
        addTriadAndProcess(createTriad(valueX, valueY, valueZ, getDefaultUnit()));
    }

    /**
     * Adds a triad of measurement samples and processes current window.
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param valueX x coordinate of measurement to be added and processed.
     * @param valueY y coordinate of measurement to be added and processed.
     * @param valueZ z coordinate of measurement to be added and processed.
     * @throws LockedException if estimator is currently running.
     */
    public void addTriadAndProcess(final M valueX, final M valueY, final M valueZ) throws LockedException {
        addTriadAndProcess(createTriad(valueX, valueY, valueZ));
    }

    /**
     * Adds a triad of measurement samples without processing current window or updating
     * result values.
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param triad measurement triad to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addTriad(final T triad) throws LockedException {
        internalAdd(triad, false);
    }

    /**
     * Adds a triad of measurement samples without processing current window or updating
     * result values.
     * Values are expressed in measurement default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param valueX x coordinate of measurement to be added.
     * @param valueY y coordinate of measurement to be added.
     * @param valueZ z coordinate of measurement to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addTriad(final double valueX, final double valueY, final double valueZ) throws LockedException {
        addTriad(createTriad(valueX, valueY, valueZ, getDefaultUnit()));
    }

    /**
     * Adds a triad of measurement samples without processing current window or updating
     * result values.
     * Notice that if there are less than {@link #getWindowSize()} processed
     * samples in the window, the remaining ones are considered to be zero
     * when average values and standard deviation is estimated.
     *
     * @param valueX x coordinate of measurement to be added.
     * @param valueY y coordinate of measurement to be added.
     * @param valueZ z coordinate of measurement to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addTriad(final M valueX, final M valueY, final M valueZ) throws LockedException {
        addTriad(createTriad(valueX, valueY, valueZ));
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

        windowedSamples.clear();
        avgX = 0.0;
        avgY = 0.0;
        avgZ = 0.0;
        varianceX = 0.0;
        varianceY = 0.0;
        varianceZ = 0.0;
        numberOfProcessedSamples = 0;

        if (listener != null) {
            //noinspection unchecked
            listener.onReset((E) this);
        }

        return true;
    }

    /**
     * Creates a copy of a triad.
     *
     * @param input triad to be copied.
     * @return copy of a triad.
     */
    protected abstract T copyTriad(final T input);

    /**
     * Creates a triad with provided values and unit.
     *
     * @param valueX x coordinate value.
     * @param valueY y coordinate value.
     * @param valueZ z coordinate value.
     * @param unit   unit.
     * @return created triad.
     */
    protected abstract T createTriad(final double valueX, final double valueY, final double valueZ, final U unit);

    /**
     * Creates a triad with provided values.
     *
     * @param valueX x coordinate value.
     * @param valueY y coordinate value.
     * @param valueZ z coordinate value.
     * @return created triad.
     */
    protected abstract T createTriad(final M valueX, final M valueY, final M valueZ);

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
     * Internally adds a triad of measurement samples and processes current window if indicated.
     *
     * @param triad   measurement triad to be added.
     * @param process true if window of samples must also be processed, false otherwise.
     * @throws LockedException if estimator is currently running.
     */
    private void internalAdd(final T triad, final boolean process) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        running = true;

        if (windowedSamples.isEmpty() && listener != null) {
            //noinspection unchecked
            listener.onStart((E) this);
        }

        final var wasFilled = isWindowFilled();
        if (wasFilled) {
            // remove first sample
            windowedSamples.removeFirst();
        }

        windowedSamples.addLast(copyTriad(triad));

        // process window
        if (process) {
            processWindow();
        }

        running = false;

        if (listener != null) {
            //noinspection unchecked
            listener.onTriadAdded((E) this);

            if (!wasFilled && isWindowFilled()) {
                //noinspection unchecked
                listener.onWindowFilled((E) this);
            }
        }
    }

    /**
     * Processes current windowed samples.
     */
    private void processWindow() {

        numberOfProcessedSamples++;

        // compute averages
        var averageX = 0.0;
        var averageY = 0.0;
        var averageZ = 0.0;
        for (final var triad : windowedSamples) {
            final var valueX = triad.getValueX();
            final var valueY = triad.getValueY();
            final var valueZ = triad.getValueZ();

            averageX += valueX;
            averageY += valueY;
            averageZ += valueZ;
        }

        averageX /= windowSize;
        averageY /= windowSize;
        averageZ /= windowSize;

        // compute variances
        var varX = 0.0;
        var varY = 0.0;
        var varZ = 0.0;
        for (final var triad : windowedSamples) {
            final var fx = triad.getValueX();
            final var fy = triad.getValueY();
            final var fz = triad.getValueZ();

            final var diffX = fx - averageX;
            final var diffY = fy - averageY;
            final var diffZ = fz - averageZ;

            final var diffX2 = diffX * diffX;
            final var diffY2 = diffY * diffY;
            final var diffZ2 = diffZ * diffZ;

            varX += diffX2;
            varY += diffY2;
            varZ += diffZ2;
        }

        final var m = windowSize - 1;

        varX /= m;
        varY /= m;
        varZ /= m;

        this.avgX = averageX;
        this.avgY = averageY;
        this.avgZ = averageZ;

        varianceX = varX;
        varianceY = varY;
        varianceZ = varZ;
    }
}
