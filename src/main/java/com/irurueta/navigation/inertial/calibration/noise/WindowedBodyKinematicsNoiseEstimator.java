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
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.GyroscopeNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.LinkedList;

/**
 * Estimates accelerometer and angular speed noise variances and PSD's
 * (Power Spectral Densities) along with their average values for a windowed
 * amount of samples.
 * This estimator must be used when the body where the accelerometer and
 * gyroscope are attached remains static on the same position with zero
 * velocity and constant (or zero) angular speed while capturing data.
 * To compute PSD's, this estimator assumes that measurement samples are obtained
 * at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, sampling rate average can be estimated using
 * {@link TimeIntervalEstimator}.
 * This estimator does NOT require the knowledge of current location and body
 * orientation.
 * Because body location and orientation is not known, estimated average values
 * cannot be used to determine biases. Only norm of noise estimations
 * (variance or standard deviation) can be safely used.
 */
@SuppressWarnings("DuplicatedCode")
public class WindowedBodyKinematicsNoiseEstimator implements AccelerometerNoiseRootPsdSource,
        GyroscopeNoiseRootPsdSource {
    /**
     * Number of samples to keep within the window by default.
     * For an accelerometer generating 100 samples/second, this is equivalent to
     * 1 second.
     * For an accelerometer generating 50 samples/second, this is equivalent to
     * 2 seconds.
     */
    public static final int DEFAULT_WINDOW_SIZE = WindowedTriadNoiseEstimator.DEFAULT_WINDOW_SIZE;

    /**
     * Minimum allowed window size.
     */
    public static final int MIN_WINDOW_SIZE = WindowedTriadNoiseEstimator.MIN_WINDOW_SIZE;

    /**
     * Default time interval between accelerometer samples expressed in seconds
     * (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS =
            WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value.
     */
    private int windowSize = DEFAULT_WINDOW_SIZE;

    /**
     * Time interval expressed in seconds (s) between consecutive accelerometer
     * samples.
     */
    private double timeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Keeps the list of body kinematics samples that remain within the window.
     */
    private final LinkedList<BodyKinematics> windowedSamples = new LinkedList<>();

    /**
     * Listener to handle events raised by this estimator.
     */
    private WindowedBodyKinematicsNoiseEstimatorListener listener;

    /**
     * Estimated average of x coordinate of specific force expressed in
     * meters per squared second (m/s^2).
     */
    private double avgSpecificForceX;

    /**
     * Estimated average of y coordinate of specific force expressed in
     * meters per squared second (m/s^2).
     */
    private double avgSpecificForceY;

    /**
     * Estimated average of z coordinate of specific force expressed in
     * meters per squared second (m/s^2).
     */
    private double avgSpecificForceZ;

    /**
     * Estimated average of x coordinate of angular rate expressed in
     * radians per second (rad/s).
     */
    private double avgAngularRateX;

    /**
     * Estimated average of y coordinate of angular rate expressed in
     * radians per second (rad/s).
     */
    private double avgAngularRateY;

    /**
     * Estimated average of z coordinate of angular rate expressed in
     * radians per second (rad/s).
     */
    private double avgAngularRateZ;

    /**
     * Estimated variance of x coordinate of specific force expressed
     * in (m^2/s^4).
     */
    private double varianceSpecificForceX;

    /**
     * Estimated variance of y coordinate of specific force expressed
     * in (m^2/s^4).
     */
    private double varianceSpecificForceY;

    /**
     * Estimated variance of z coordinate of specific force expressed
     * in (m^2/s^4).
     */
    private double varianceSpecificForceZ;

    /**
     * Estimated variance of x coordinate of angular rate expressed
     * in (rad^2/s^2).
     */
    private double varianceAngularRateX;

    /**
     * Estimated variance of y coordinate of angular rate expressed
     * in (rad^2/s^2).
     */
    private double varianceAngularRateY;

    /**
     * Estimated variance of z coordinate of angular rate expressed
     * in (rad^2/s^2).
     */
    private double varianceAngularRateZ;

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
    public WindowedBodyKinematicsNoiseEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public WindowedBodyKinematicsNoiseEstimator(final WindowedBodyKinematicsNoiseEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Gets length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value.
     *
     * @return length of number of samples to keep within the window.
     */
    public int getWindowSize() {
        return windowSize;
    }

    /**
     * Sets length of number of samples to keep within the window being processed.
     * Window size must always be larger than allowed minimum value.
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

        this.windowSize = windowSize;
        reset();
    }

    /**
     * Gets time interval between body kinematics samples expressed in
     * seconds (s).
     *
     * @return time interval between accelerometer triad samples.
     */
    public double getTimeInterval() {
        return timeInterval;
    }

    /**
     * Sets time interval between body kinematics samples expressed in
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
     * Gets time interval between body kinematics samples.
     *
     * @return time interval between accelerometer triad samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(timeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between body kinematics samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(timeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between body kinematics samples.
     *
     * @param timeInterval time interval between accelerometer triad samples.
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
    public WindowedBodyKinematicsNoiseEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final WindowedBodyKinematicsNoiseEstimatorListener listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets first provided body kinematics within the window.
     *
     * @return first provided body kinematics within the window or null if not
     * available.
     */
    public BodyKinematics getFirstWindowedBodyKinematics() {
        return windowedSamples.isEmpty() ? null : windowedSamples.getFirst();
    }

    /**
     * Gets first provided body kinematics within the window.
     *
     * @param result instance where first provided body kinematics will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getFirstWindowedBodyKinematics(final BodyKinematics result) {
        if (windowedSamples.isEmpty()) {
            return false;
        } else {
            result.copyFrom(windowedSamples.getFirst());
            return true;
        }
    }

    /**
     * Gets last provided body kinematics within the window.
     *
     * @return last provided body kinematics within the window or null if not
     * available.
     */
    public BodyKinematics getLastWindowedBodyKinematics() {
        return windowedSamples.isEmpty() ? null : windowedSamples.getLast();
    }

    /**
     * Gets last provided body kinematics within the window.
     *
     * @param result instance where last provided body kinematics will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getLastWindowedBodyKinematics(final BodyKinematics result) {
        if (windowedSamples.isEmpty()) {
            return false;
        } else {
            result.copyFrom(windowedSamples.getLast());
            return true;
        }
    }

    /**
     * Gets estimated average of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed specific force.
     */
    public double getAvgSpecificForceX() {
        return avgSpecificForceX;
    }

    /**
     * Gets estimated average of x coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed specific force.
     */
    public Acceleration getAvgSpecificForceXAsMeasurement() {
        return new Acceleration(avgSpecificForceX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of x coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of x coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgSpecificForceXAsMeasurement(final Acceleration result) {
        result.setValue(avgSpecificForceX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed specific force.
     */
    public double getAvgSpecificForceY() {
        return avgSpecificForceY;
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed specific force.
     */
    public Acceleration getAvgSpecificForceYAsMeasurement() {
        return new Acceleration(avgSpecificForceY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of y coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of y coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgSpecificForceYAsMeasurement(final Acceleration result) {
        result.setValue(avgSpecificForceY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed specific force.
     */
    public double getAvgSpecificForceZ() {
        return avgSpecificForceZ;
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed specific force.
     */
    public Acceleration getAvgSpecificForceZAsMeasurement() {
        return new Acceleration(avgSpecificForceZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of z coordinate of accelerometer sensed specific force.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of z coordinate of sensed specific force
     *               will be stored.
     */
    public void getAvgSpecificForceZAsMeasurement(final Acceleration result) {
        result.setValue(avgSpecificForceZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of accelerometer sensed specific force as a measurement
     * triad.
     *
     * @return average accelerometer triad.
     */
    public AccelerationTriad getAvgSpecificForceAsTriad() {
        return new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                avgSpecificForceX, avgSpecificForceY, avgSpecificForceZ);
    }

    /**
     * Gets estimated average of accelerometer sensed specific force as a measurement
     * triad.
     *
     * @param result instance where average accelerometer triad will be stored.
     */
    public void getAvgSpecificForceAsTriad(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(avgSpecificForceX, avgSpecificForceY, avgSpecificForceZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets norm of estimated average acceleration expressed in meters per squared
     * second (m/s^2). This value is independent of body orientation.
     *
     * @return norm of estimated average acceleration.
     */
    public double getAvgSpecificForceNorm() {
        return Math.sqrt(avgSpecificForceX * avgSpecificForceX + avgSpecificForceY * avgSpecificForceY
                + avgSpecificForceZ * avgSpecificForceZ);
    }

    /**
     * Gets norm of estimated average acceleration within current window.
     *
     * @return norm of estimated average acceleration.
     */
    public Acceleration getAvgSpecificForceNormAsMeasurement() {
        return new Acceleration(getAvgSpecificForceNorm(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets norm of estimated average acceleration.
     *
     * @param result instance where norm of estimated average acceleration will be stored.
     */
    public void getAvgSpecificForceNormAsMeasurement(final Acceleration result) {
        result.setValue(getAvgSpecificForceNorm());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     * This value will depend of body location and orientation, hence it should
     * never be used as a calibration bias.
     *
     * @return average of x coordinate of sensed angular rate.
     */
    public double getAvgAngularRateX() {
        return avgAngularRateX;
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of x coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateXAsMeasurement() {
        return new AngularSpeed(avgAngularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of x coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of x coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateXAsMeasurement(final AngularSpeed result) {
        result.setValue(avgAngularRateX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of y coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed angular rate.
     */
    public double getAvgAngularRateY() {
        return avgAngularRateY;
    }

    /**
     * Gets estimated average of y coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of y coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateYAsMeasurement() {
        return new AngularSpeed(avgAngularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of y coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of y coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateYAsMeasurement(final AngularSpeed result) {
        result.setValue(avgAngularRateY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of z coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed angular rate.
     */
    public double getAvgAngularRateZ() {
        return avgAngularRateZ;
    }

    /**
     * Gets estimated average of z coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @return average of z coordinate of sensed angular rate.
     */
    public AngularSpeed getAvgAngularRateZAsMeasurement() {
        return new AngularSpeed(avgAngularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of z coordinate of gyroscope sensed angular rate.
     * This value will depend of body location and orientation, hence it should never
     * be used as a calibration bias.
     *
     * @param result instance where average of z coordinate of sensed angular rate
     *               will be stored.
     */
    public void getAvgAngularRateZAsMeasurement(final AngularSpeed result) {
        result.setValue(avgAngularRateZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of gyroscope sensed angular speed as a measurement
     * triad.
     *
     * @return average angular speed triad.
     */
    public AngularSpeedTriad getAvgAngularRateTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                avgAngularRateX, avgAngularRateY, avgAngularRateZ);
    }

    /**
     * Gets estimated average of gyroscope sensed angular speed as a measurement
     * triad.
     *
     * @param result instance where average angular speed triad will be stored.
     */
    public void getAvgAngularRateTriad(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(avgAngularRateX, avgAngularRateY, avgAngularRateZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets norm of estimated average angular speed expressed in radians per
     * second (rad/s). This value is independent of body orientation.
     *
     * @return norm of estimated average angular speed.
     */
    public double getAvgAngularRateNorm() {
        return Math.sqrt(avgAngularRateX * avgAngularRateX
                + avgAngularRateY * avgAngularRateY
                + avgAngularRateZ * avgAngularRateZ);
    }

    /**
     * Gets norm of estimated average angular speed.
     * This value is independent of body orientation.
     *
     * @return norm of estimated average angular speed.
     */
    public AngularSpeed getAvgAngularRateNormAsMeasurement() {
        return new AngularSpeed(getAvgAngularRateNorm(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets norm of estimated average angular speed.
     * This value is independent of body orientation.
     *
     * @param result instance where norm of estimated average angular speed will be stored.
     */
    public void getAvgAngularRateNormAsMeasurement(final AngularSpeed result) {
        result.setValue(getAvgAngularRateNorm());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated average of body kinematics.
     *
     * @return estimated average of body kinematics.
     */
    public BodyKinematics getAvgBodyKinematics() {
        final var result = new BodyKinematics();
        getAvgBodyKinematics(result);
        return result;
    }

    /**
     * Gets estimated average of body kinematics.
     *
     * @param result instance where estimated average of body kinematics will be stored.
     */
    public void getAvgBodyKinematics(final BodyKinematics result) {
        result.setSpecificForceCoordinates(getAvgSpecificForceX(), getAvgSpecificForceY(), getAvgSpecificForceZ());
        result.setAngularRateCoordinates(getAvgAngularRateX(), getAvgAngularRateY(), getAvgAngularRateZ());
    }

    /**
     * Gets estimated variance of x coordinate of accelerometer sensed specific force
     * expressed in (m^2/s^4).
     *
     * @return estimated variance of x coordinate of sensed specific force.
     */
    public double getVarianceSpecificForceX() {
        return varianceSpecificForceX;
    }

    /**
     * Gets estimated variance of y coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of y coordinate of sensed specific force.
     */
    public double getVarianceSpecificForceY() {
        return varianceSpecificForceY;
    }

    /**
     * Gets estimated variance of z coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of z coordinate of sensed specific force.
     */
    public double getVarianceSpecificForceZ() {
        return varianceSpecificForceZ;
    }

    /**
     * Gets estimated variance of x coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of x coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateX() {
        return varianceAngularRateX;
    }

    /**
     * Gets estimated variance of y coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of y coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateY() {
        return varianceAngularRateY;
    }

    /**
     * Gets estimated variance of z coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     *
     * @return estimated variance of z coordinate of sensed angular rate.
     */
    public double getVarianceAngularRateZ() {
        return varianceAngularRateZ;
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force expressed in meters per squared second (m/s^2).
     *
     * @return estimated standard deviation of x coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationSpecificForceX() {
        return Math.sqrt(varianceSpecificForceX);
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of x coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationSpecificForceXAsMeasurement() {
        return new Acceleration(getStandardDeviationSpecificForceX(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of x
     *               coordinate of sensed specific force will be stored.
     */
    public void getStandardDeviationSpecificForceXAsMeasurement(final Acceleration result) {
        result.setValue(getStandardDeviationSpecificForceX());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force expressed in meters per squared second (m/s^2).
     *
     * @return estimated standard deviation of y coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationSpecificForceY() {
        return Math.sqrt(varianceSpecificForceY);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of y coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationSpecificForceYAsMeasurement() {
        return new Acceleration(getStandardDeviationSpecificForceY(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of y
     *               coordinate of sensed specific force will be stored.
     */
    public void getStandardDeviationSpecificForceYAsMeasurement(final Acceleration result) {
        result.setValue(getStandardDeviationSpecificForceY());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force expressed in meters per squared second (m/s^2).
     *
     * @return estimated standard deviation of z coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationSpecificForceZ() {
        return Math.sqrt(varianceSpecificForceZ);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of z coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationSpecificForceZAsMeasurement() {
        return new Acceleration(getStandardDeviationSpecificForceZ(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of z
     *               coordinate of sensed specific force will be stored.
     */
    public void getStandardDeviationSpecificForceZAsMeasurement(final Acceleration result) {
        result.setValue(getStandardDeviationSpecificForceZ());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation triad of accelerometer measurements.
     *
     * @return estimated standard deviation triad of accelerometer measurements.
     */
    public AccelerationTriad getStandardDeviationSpecificForceTriad() {
        return new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                getStandardDeviationSpecificForceX(),
                getStandardDeviationSpecificForceY(),
                getStandardDeviationSpecificForceZ());
    }

    /**
     * Gets estimated standard deviation triad of accelerometer measurements.
     *
     * @param result instance where estimated standard deviation triad of
     *               accelerometer measurements will be stored.
     */
    public void getStandardDeviationSpecificForceTriad(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(getStandardDeviationSpecificForceX(),
                getStandardDeviationSpecificForceY(),
                getStandardDeviationSpecificForceZ(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer measurements
     * expressed in meters per squared second (m/s^2).
     *
     * @return norm of estimated standard deviation of accelerometer
     * measurements.
     */
    public double getStandardDeviationSpecificForceNorm() {
        final var fx = getStandardDeviationSpecificForceX();
        final var fy = getStandardDeviationSpecificForceY();
        final var fz = getStandardDeviationSpecificForceZ();
        return Math.sqrt(fx * fx + fy * fy + fz * fz);
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer measurements.
     *
     * @return norm of estimated standard deviation of measurements.
     */
    public Acceleration getStandardDeviationSpecificForceNormAsMeasurement() {
        return new Acceleration(getStandardDeviationSpecificForceNorm(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer measurements.
     *
     * @param result instance where norm of estimated standard deviation will be
     *               stored.
     */
    public void getStandardDeviationSpecificForceNormAsMeasurement(final Acceleration result) {
        result.setValue(getStandardDeviationSpecificForceNorm());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets average of estimated standard deviation coordinates of accelerometer
     * measurements expressed in meters per squared second (m/s^2).
     *
     * @return average of estimated standard deviation coordinates.
     */
    public double getAverageStandardDeviationSpecificForce() {
        final var fx = getStandardDeviationSpecificForceX();
        final var fy = getStandardDeviationSpecificForceY();
        final var fz = getStandardDeviationSpecificForceZ();
        return (fx + fy + fz) / 3.0;
    }

    /**
     * Gets average of estimated standard deviation coordinates of accelerometer
     * measurements.
     *
     * @return average of estimated standard deviation coordinates.
     */
    public Acceleration getAverageStandardDeviationSpecificForceAsMeasurement() {
        return new Acceleration(getAverageStandardDeviationSpecificForce(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets average of estimated standard deviation coordinates of accelerometer
     * measurements.
     *
     * @param result instance where average of estimated standard deviation coordinates
     *               will be stored.
     */
    public void getAverageStandardDeviationSpecificForceAsMeasurement(final Acceleration result) {
        result.setValue(getAverageStandardDeviationSpecificForce());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope
     * expressed in radians per second (rad/s).
     *
     * @return estimated standard deviation of x coordinate of gyroscope.
     */
    public double getStandardDeviationAngularRateX() {
        return Math.sqrt(varianceAngularRateX);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope.
     *
     * @return estimated standard deviation of x coordinate of gyroscope.
     */
    public AngularSpeed getStandardDeviationAngularRateXAsMeasurement() {
        return new AngularSpeed(getStandardDeviationAngularRateX(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope.
     *
     * @param result estimated standard deviation of x coordinate of gyroscope.
     */
    public void getStandardDeviationAngularRateXAsMeasurement(final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateX());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope
     * expressed in radians per second (rad/s).
     *
     * @return estimated standard deviation of y coordinate of gyroscope.
     */
    public double getStandardDeviationAngularRateY() {
        return Math.sqrt(varianceAngularRateY);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope.
     *
     * @return estimated standard deviation of y coordinate of gyroscope.
     */
    public AngularSpeed getStandardDeviationAngularRateYAsMeasurement() {
        return new AngularSpeed(getStandardDeviationAngularRateY(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope.
     *
     * @param result estimated standard deviation of y coordinate of gyroscope.
     */
    public void getStandardDeviationAngularRateYAsMeasurement(final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateY());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope
     * expressed in radians per second (rad/s).
     *
     * @return estimated standard deviation of z coordinate of gyroscope.
     */
    public double getStandardDeviationAngularRateZ() {
        return Math.sqrt(varianceAngularRateZ);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope.
     *
     * @return estimated standard deviation of z coordinate of gyroscope.
     */
    public AngularSpeed getStandardDeviationAngularRateZAsMeasurement() {
        return new AngularSpeed(getStandardDeviationAngularRateZ(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope.
     *
     * @param result estimated standard deviation of z coordinate of gyroscope.
     */
    public void getStandardDeviationAngularRateZAsMeasurement(final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateZ());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation triad of angular speed measurements.
     *
     * @return estimated standard deviation triad of angular speed measurements.
     */
    public AngularSpeedTriad getStandardDeviationAngularSpeedTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                getStandardDeviationAngularRateX(),
                getStandardDeviationAngularRateY(),
                getStandardDeviationAngularRateZ());
    }

    /**
     * Gets estimated standard deviation triad of angular speed measurements.
     *
     * @param result instance where estimated standard deviation triad of
     *               gyroscope measurements will be stored.
     */
    public void getStandardDeviationAngularSpeedTriad(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(getStandardDeviationAngularRateX(),
                getStandardDeviationAngularRateY(),
                getStandardDeviationAngularRateZ(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope measurements
     * expressed in radians per second (rad/s).
     *
     * @return norm of estimated standard deviation of gyroscope
     * measurements.
     */
    public double getStandardDeviationAngularSpeedNorm() {
        final var wx = getStandardDeviationAngularRateX();
        final var wy = getStandardDeviationAngularRateY();
        final var wz = getStandardDeviationAngularRateZ();
        return Math.sqrt(wx * wx + wy * wy + wz * wz);
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope measurements.
     *
     * @return norm of estimated standard deviation of measurements.
     */
    public AngularSpeed getStandardDeviationAngularSpeedNormAsMeasurement() {
        return new AngularSpeed(getStandardDeviationAngularSpeedNorm(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope measurements.
     *
     * @param result instance where norm of estimated standard deviation will be
     *               stored.
     */
    public void getStandardDeviationAngularSpeedNormAsMeasurement(final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularSpeedNorm());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets average of estimated standard deviation coordinates of gyroscope
     * measurements expressed in radians per second (rad/s).
     *
     * @return average of estimated standard deviation coordinates.
     */
    public double getAverageStandardDeviationAngularSpeed() {
        final var wx = getStandardDeviationAngularRateX();
        final var wy = getStandardDeviationAngularRateY();
        final var wz = getStandardDeviationAngularRateZ();
        return (wx + wy + wz) / 3.0;
    }

    /**
     * Gets average of estimated standard deviation coordinates of gyroscope
     * measurements.
     *
     * @return average of estimated standard deviation coordinates.
     */
    public AngularSpeed getAverageStandardDeviationAngularSpeedAsMeasurement() {
        return new AngularSpeed(getAverageStandardDeviationAngularSpeed(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets average of estimated standard deviation coordinates of gyroscope
     * measurements.
     *
     * @param result instance where average of estimated standard deviation coordinates
     *               will be stored.
     */
    public void getAverageStandardDeviationAngularSpeedAsMeasurement(final AngularSpeed result) {
        result.setValue(getAverageStandardDeviationAngularSpeed());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviations of accelerometer and gyroscope components
     * as a body kinematics instance.
     *
     * @return a body kinematics instance containing standard deviation values.
     */
    public BodyKinematics getStandardDeviationAsBodyKinematics() {
        return new BodyKinematics(getStandardDeviationSpecificForceX(),
                getStandardDeviationSpecificForceY(),
                getStandardDeviationSpecificForceZ(),
                getStandardDeviationAngularRateX(),
                getStandardDeviationAngularRateY(),
                getStandardDeviationAngularRateZ());
    }

    /**
     * Gets estimated standard deviations of accelerometer and gyroscope components
     * as a body kinematics instance.
     *
     * @param result instance where data will be stored.
     */
    public void getStandardDeviationAsBodyKinematics(final BodyKinematics result) {
        result.setSpecificForceCoordinates(getStandardDeviationSpecificForceX(),
                getStandardDeviationSpecificForceY(),
                getStandardDeviationSpecificForceZ());
        result.setAngularRateCoordinates(getStandardDeviationAngularRateX(),
                getStandardDeviationAngularRateY(),
                getStandardDeviationAngularRateZ());
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on x axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on x axis.
     */
    public double getSpecificForcePsdX() {
        return varianceSpecificForceX * timeInterval;
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on y axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on y axis.
     */
    public double getSpecificForcePsdY() {
        return varianceSpecificForceY * timeInterval;
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on z axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on z axis.
     */
    public double getSpecificForcePsdZ() {
        return varianceSpecificForceZ * timeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on x axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on x axis.
     */
    public double getAngularRatePsdX() {
        return varianceAngularRateX * timeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on y axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on y axis.
     */
    public double getAngularRatePsdY() {
        return varianceAngularRateY * timeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on z axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on z axis.
     */
    public double getAngularRatePsdZ() {
        return varianceAngularRateZ * timeInterval;
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on x axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on x axis.
     */
    public double getSpecificForceRootPsdX() {
        return Math.sqrt(getSpecificForcePsdX());
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on y axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on y axis.
     */
    public double getSpecificForceRootPsdY() {
        return Math.sqrt(getSpecificForcePsdY());
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on z axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on z axis.
     */
    public double getSpecificForceRootPsdZ() {
        return Math.sqrt(getSpecificForcePsdZ());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on x axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on x axis.
     */
    public double getAngularRateRootPsdX() {
        return Math.sqrt(getAngularRatePsdX());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on y axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on y axis.
     */
    public double getAngularRateRootPsdY() {
        return Math.sqrt(getAngularRatePsdY());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on z axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on z axis.
     */
    public double getAngularRateRootPsdZ() {
        return Math.sqrt(getAngularRatePsdZ());
    }

    /**
     * Gets average accelerometer noise PSD (Power Spectral Density) among
     * x,y,z components expressed as (m^2/s^-3).
     *
     * @return average accelerometer noise PSD.
     */
    public double getAvgSpecificForceNoisePsd() {
        return (getSpecificForcePsdX() + getSpecificForcePsdY() + getSpecificForcePsdZ()) / 3.0;
    }

    /**
     * Gets norm of noise root PSD (Power Spectral Density) among x,y,z
     * components expressed as (m * s^-1.5).
     *
     * @return norm of noise root PSD.
     */
    public double getSpecificForceNoiseRootPsdNorm() {
        return Math.sqrt(getSpecificForcePsdX() + getSpecificForcePsdY() + getSpecificForcePsdZ());
    }

    /**
     * Gets average gyroscope noise PSD (Power Spectral Density) among
     * x,y,z components expressed in (rad^2/s).
     *
     * @return average gyroscope noise PSD.
     */
    public double getAvgAngularRateNoisePsd() {
        return (getAngularRatePsdX() + getAngularRatePsdY() + getAngularRatePsdZ()) / 3.0;
    }

    /**
     * Gets norm of noise root PSD (Power Spectral Density) among x,y,z
     * components expressed as (rad * s^-0.5).
     *
     * @return norm of noise root PSD.
     */
    public double getAngularRateNoiseRootPsdNorm() {
        return Math.sqrt(getAngularRatePsdX() + getAngularRatePsdY() + getAngularRatePsdZ());
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
     * Adds a body kinematics measurement and processes current window.
     *
     * @param specificForceX x coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param specificForceY y coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param specificForceZ z coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param angularRateX   x coordinate of angular rate expressed in radians per second (rad/s).
     * @param angularRateY   y coordinate of angular rate expressed in radians per second (rad/s).
     * @param angularRateZ   z coordinate of angular rate expressed in radians per second (rad/s).
     * @return true if provided kinematics instance has been processed, false if it has
     * been ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addBodyKinematicsAndProcess(
            final double specificForceX, final double specificForceY, final double specificForceZ,
            final double angularRateX, final double angularRateY, final double angularRateZ) throws LockedException {
        return addBodyKinematicsAndProcess(new BodyKinematics(specificForceX, specificForceY, specificForceZ,
                angularRateX, angularRateY, angularRateZ));
    }

    /**
     * Adds a body kinematics measurement and processes current window.
     *
     * @param specificForceX x coordinate of specific force.
     * @param specificForceY y coordinate of specific force.
     * @param specificForceZ z coordinate of specific force.
     * @param angularRateX   x coordinate of angular rate.
     * @param angularRateY   y coordinate of angular rate.
     * @param angularRateZ   z coordinate of angular rate.
     * @return true if provided kinematics instance has been processed, false if it has
     * been ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addBodyKinematicsAndProcess(
            final Acceleration specificForceX, final Acceleration specificForceY, final Acceleration specificForceZ,
            final AngularSpeed angularRateX, final AngularSpeed angularRateY, final AngularSpeed angularRateZ)
            throws LockedException {
        return addBodyKinematicsAndProcess(new BodyKinematics(specificForceX, specificForceY, specificForceZ,
                angularRateX, angularRateY, angularRateZ));
    }

    /**
     * Adds a body kinematics measurement and processes current window.
     *
     * @param specificForce specific force triad.
     * @param angularSpeed  angular speed triad.
     * @return true if provided kinematics instance has been processed, false if it has
     * been ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addBodyKinematicsAndProcess(
            final AccelerationTriad specificForce, final AngularSpeedTriad angularSpeed) throws LockedException {
        return addBodyKinematicsAndProcess(new BodyKinematics(specificForce, angularSpeed));
    }

    /**
     * Adds a body kinematics measurement and processes current window.
     *
     * @param kinematics body kinematics to be added and processed.
     * @return true if provided kinematics instance has been processed, false if it has
     * been ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addBodyKinematicsAndProcess(final BodyKinematics kinematics) throws LockedException {
        return internalAdd(kinematics, true);
    }

    /**
     * Adds a body kinematics measurement.
     *
     * @param specificForceX x coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param specificForceY y coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param specificForceZ z coordinate of specific force expressed in meters per squared second (m/s^2).
     * @param angularRateX   x coordinate of angular rate expressed in radians per second (rad/s).
     * @param angularRateY   y coordinate of angular rate expressed in radians per second (rad/s).
     * @param angularRateZ   z coordinate of angular rate expressed in radians per second (rad/s).
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(
            final double specificForceX, final double specificForceY, final double specificForceZ,
            final double angularRateX, final double angularRateY, final double angularRateZ) throws LockedException {
        addBodyKinematics(new BodyKinematics(specificForceX, specificForceY, specificForceZ,
                angularRateX, angularRateY, angularRateZ));
    }

    /**
     * Adds a body kinematics measurement.
     *
     * @param specificForceX x coordinate of specific force.
     * @param specificForceY y coordinate of specific force.
     * @param specificForceZ z coordinate of specific force.
     * @param angularRateX   x coordinate of angular rate.
     * @param angularRateY   y coordinate of angular rate.
     * @param angularRateZ   z coordinate of angular rate.
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(
            final Acceleration specificForceX, final Acceleration specificForceY, final Acceleration specificForceZ,
            final AngularSpeed angularRateX, final AngularSpeed angularRateY, final AngularSpeed angularRateZ)
            throws LockedException {
        addBodyKinematics(new BodyKinematics(specificForceX, specificForceY, specificForceZ,
                angularRateX, angularRateY, angularRateZ));
    }

    /**
     * Adds a body kinematics measurement.
     *
     * @param specificForce specific force triad.
     * @param angularSpeed  angular speed triad.
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(
            final AccelerationTriad specificForce, final AngularSpeedTriad angularSpeed) throws LockedException {
        addBodyKinematics(new BodyKinematics(specificForce, angularSpeed));
    }

    /**
     * Adds a body kinematics measurement.
     *
     * @param kinematics body kinematics to be added.
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(final BodyKinematics kinematics) throws LockedException {
        internalAdd(kinematics, false);
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
        avgSpecificForceX = 0.0;
        avgSpecificForceY = 0.0;
        avgSpecificForceZ = 0.0;
        avgAngularRateX = 0.0;
        avgAngularRateY = 0.0;
        avgAngularRateZ = 0.0;
        varianceSpecificForceX = 0.0;
        varianceSpecificForceY = 0.0;
        varianceSpecificForceZ = 0.0;
        varianceAngularRateX = 0.0;
        varianceAngularRateY = 0.0;
        varianceAngularRateZ = 0.0;
        numberOfProcessedSamples = 0;

        if (listener != null) {
            listener.onReset(this);
        }

        return true;
    }

    /**
     * Internally adds a body kinematics measurement and processes current window if indicated.
     *
     * @param kinematics body kinematics to be added.
     * @param process    true if window of samples must also be processed, false otherwise.
     * @return true if result values were updated, false if not enough samples are available yet
     * and no average or variance values have been computed yet.
     * @throws LockedException if estimator is currently running.
     */
    private boolean internalAdd(final BodyKinematics kinematics, boolean process) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        running = true;

        if (windowedSamples.isEmpty() && listener != null) {
            listener.onStart(this);
        }

        final var wasFilled = isWindowFilled();
        if (wasFilled) {
            // remove first sample
            windowedSamples.removeFirst();
        }

        windowedSamples.addLast(new BodyKinematics(kinematics));

        // process window
        final var result = process && processWindow();

        running = false;

        if (listener != null) {
            listener.onBodyKinematicsAdded(this);

            if (!wasFilled && isWindowFilled()) {
                listener.onWindowFilled(this);
            }
        }

        return result;
    }

    /**
     * Processes current windowed samples.
     *
     * @return true if sample was processed, false it there are not enough samples to
     * process current window.
     */
    private boolean processWindow() {
        numberOfProcessedSamples++;

        final var n = windowedSamples.size();
        if (n <= 1) {
            return false;
        }

        // compute averages
        var avgFx = 0.0;
        var avgFy = 0.0;
        var avgFz = 0.0;
        var avgWx = 0.0;
        var avgWy = 0.0;
        var avgWz = 0.0;
        for (final var kinematics : windowedSamples) {
            final var fx = kinematics.getFx();
            final var fy = kinematics.getFy();
            final var fz = kinematics.getFz();
            final var wx = kinematics.getAngularRateX();
            final var wy = kinematics.getAngularRateY();
            final var wz = kinematics.getAngularRateZ();

            avgFx += fx;
            avgFy += fy;
            avgFz += fz;
            avgWx += wx;
            avgWy += wy;
            avgWz += wz;
        }

        avgFx /= n;
        avgFy /= n;
        avgFz /= n;
        avgWx /= n;
        avgWy /= n;
        avgWz /= n;

        // compute variances
        var varFx = 0.0;
        var varFy = 0.0;
        var varFz = 0.0;
        var varWx = 0.0;
        var varWy = 0.0;
        var varWz = 0.0;
        for (final var kinematics : windowedSamples) {
            final var fx = kinematics.getFx();
            final var fy = kinematics.getFy();
            final var fz = kinematics.getFz();
            final var wx = kinematics.getAngularRateX();
            final var wy = kinematics.getAngularRateY();
            final var wz = kinematics.getAngularRateZ();

            final var diffFx = fx - avgFx;
            final var diffFy = fy - avgFy;
            final var diffFz = fz - avgFz;
            final var diffWx = wx - avgWx;
            final var diffWy = wy - avgWy;
            final var diffWz = wz - avgWz;

            final var diffFx2 = diffFx * diffFx;
            final var diffFy2 = diffFy * diffFy;
            final var diffFz2 = diffFz * diffFz;
            final var diffWx2 = diffWx * diffWx;
            final var diffWy2 = diffWy * diffWy;
            final var diffWz2 = diffWz * diffWz;

            varFx += diffFx2;
            varFy += diffFy2;
            varFz += diffFz2;
            varWx += diffWx2;
            varWy += diffWy2;
            varWz += diffWz2;
        }

        final var nMinusOne = n - 1;

        varFx /= nMinusOne;
        varFy /= nMinusOne;
        varFz /= nMinusOne;
        varWx /= nMinusOne;
        varWy /= nMinusOne;
        varWz /= nMinusOne;

        avgSpecificForceX = avgFx;
        avgSpecificForceY = avgFy;
        avgSpecificForceZ = avgFz;
        avgAngularRateX = avgWx;
        avgAngularRateY = avgWy;
        avgAngularRateZ = avgWz;

        varianceSpecificForceX = varFx;
        varianceSpecificForceY = varFy;
        varianceSpecificForceZ = varFz;
        varianceAngularRateX = varWx;
        varianceAngularRateY = varWy;
        varianceAngularRateZ = varWz;

        return true;
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    @Override
    public double getAccelerometerBaseNoiseLevelRootPsd() {
        return getSpecificForceNoiseRootPsdNorm();
    }

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    @Override
    public double getGyroscopeBaseNoiseLevelRootPsd() {
        return getAngularRateNoiseRootPsdNorm();
    }
}
