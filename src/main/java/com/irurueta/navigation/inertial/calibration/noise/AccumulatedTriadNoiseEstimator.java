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

/**
 * Base class to estimate measurement noise variances and PSD's (Power Spectral Densities)
 * along with their average values.
 * Implementations of this estimator must be used when the body where the measurement device
 * is attached to remains static on the same position with zero velocity, or
 * with constant angular speed and orientation while capturing data.
 * To compute PSD's, this estimator assumes that measurement samples are obtained
 * at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, accelerometer sampling rate average can be estimated using
 * {@link TimeIntervalEstimator}.
 * This estimator does NOT require the knowledge of current location and body
 * orientation.
 * Because body location and orientation is not known, estimated average values
 * cannot be used to determine biases. Only norm of noise estimations
 * (variance or standard deviation) can be safely used.
 * Notice that this estimator uses a biased variance estimator, which for a large
 * number of samples converges to the unbiased variance estimator.
 *
 * @param <U> a measurement unit type.
 * @param <M> a measurement type.
 * @param <T> a triad type.
 * @param <E> an estimator type.
 * @param <L> a listener type.
 */
public abstract class AccumulatedTriadNoiseEstimator<U extends Enum<?>,
        M extends Measurement<U>, T extends Triad<U, M>,
        E extends AccumulatedTriadNoiseEstimator<U, M, T, E, L>,
        L extends AccumulatedTriadNoiseEstimatorListener<U, M, T, E>> {

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
     * Last provided triad.
     */
    private T lastTriad;

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
    protected AccumulatedTriadNoiseEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    protected AccumulatedTriadNoiseEstimator(final L listener) {
        this.listener = listener;
    }

    /**
     * Gets time interval between triad samples expressed in
     * seconds (s).
     *
     * @return time interval between accelerometer triad samples.
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
     * Gets last provided triad values or null if not available.
     *
     * @return last provided triad values or null.
     */
    public T getLastTriad() {
        return lastTriad;
    }

    /**
     * Gets last provided triad values.
     *
     * @param result instance where last provided triad will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getLastTriad(final T result) {
        if (lastTriad != null) {
            lastTriad.copyTo(result);
            return true;
        } else {
            return false;
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
     * Gets norm of estimated average measurement expressed in its default
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
     * Gets estimated standard deviation of measurements.
     *
     * @return estimated standard deviation triad of measurements.
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
     * Gets norm of estimated standard deviation of measurement
     * expressed in its default unit (m/s^2 for acceleration, rad/s for
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
     * Gets norm of estimated standard deviation of measurements.
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
     * Gets average of estimated standard deviation coordinates of measurement
     * expressed in its default unit (m/s^2 for acceleration, rad/s for
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
     * Gets average of estimated standard deviation coordinates of measurement.
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
     * Indicates whether estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Adds a triad of measurement samples.
     * Values are expressed in measurement default unit (m/s^2 for acceleration, rad/s for
     * angular speed or T for magnetic flux density).
     *
     * @param valueX x coordinate of measurement to be added and processed.
     * @param valueY y coordinate of measurement to be added and processed.
     * @param valueZ z coordinate of measurement to be added and processed.
     * @throws LockedException if estimator is currently running.
     */
    @SuppressWarnings("DuplicatedCode")
    public void addTriad(final double valueX, final double valueY, final double valueZ) throws LockedException {

        if (running) {
            throw new LockedException();
        }

        running = true;

        if (lastTriad == null && listener != null) {
            //noinspection unchecked
            listener.onStart((E) this);
        }

        // compute averages
        final var tmp = (double) numberOfProcessedSamples / (double) numberOfProcessedSamplesPlusOne;
        avgX = avgX * tmp + valueX / numberOfProcessedSamplesPlusOne;
        avgY = avgY * tmp + valueY / numberOfProcessedSamplesPlusOne;
        avgZ = avgZ * tmp + valueZ / numberOfProcessedSamplesPlusOne;

        // compute variances
        final var diffX = valueX - avgX;
        final var diffY = valueY - avgY;
        final var diffZ = valueZ - avgZ;
        final var diffX2 = diffX * diffX;
        final var diffY2 = diffY * diffY;
        final var diffZ2 = diffZ * diffZ;

        varianceX = varianceX * tmp + diffX2 / numberOfProcessedSamplesPlusOne;
        varianceY = varianceY * tmp + diffY2 / numberOfProcessedSamplesPlusOne;
        varianceZ = varianceZ * tmp + diffZ2 / numberOfProcessedSamplesPlusOne;

        if (lastTriad == null) {
            lastTriad = createTriad(valueX, valueY, valueZ, getDefaultUnit());
        } else {
            lastTriad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, getDefaultUnit());
        }

        numberOfProcessedSamples++;
        numberOfProcessedSamplesPlusOne++;

        if (listener != null) {
            //noinspection unchecked
            listener.onTriadAdded((E) this);
        }

        running = false;
    }

    /**
     * Adds a triad of measurement samples.
     *
     * @param triad measurement triad to be added and processed.
     * @throws LockedException if estimator is currently running.
     */
    public void addTriad(final T triad) throws LockedException {
        addTriad(convertToDefaultUnit(triad.getValueX(), triad.getUnit()),
                convertToDefaultUnit(triad.getValueY(), triad.getUnit()),
                convertToDefaultUnit(triad.getValueZ(), triad.getUnit()));
    }

    /**
     * Adds a triad of measurement samples.
     *
     * @param valueX x coordinate of measurement to be added and processed.
     * @param valueY y coordinate of measurement to be added and processed.
     * @param valueZ z coordinate of measurement to be added and processed.
     * @throws LockedException if estimator is currently running.
     */
    public void addTriad(final M valueX, final M valueY, final M valueZ) throws LockedException {
        addTriad(convertToDefaultUnit(valueX.getValue().doubleValue(), valueX.getUnit()),
                convertToDefaultUnit(valueY.getValue().doubleValue(), valueY.getUnit()),
                convertToDefaultUnit(valueZ.getValue().doubleValue(), valueZ.getUnit()));
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
        lastTriad = null;
        avgX = 0.0;
        avgY = 0.0;
        avgZ = 0.0;
        varianceX = 0.0;
        varianceY = 0.0;
        varianceZ = 0.0;
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
     * Converts provided value and unit into default unit.
     *
     * @param value measurement value to be converted.
     * @param unit  unit of measurement value to be converted.
     * @return converted value.
     */
    protected abstract double convertToDefaultUnit(final double value, final U unit);
}
