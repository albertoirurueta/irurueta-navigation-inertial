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
package com.irurueta.navigation.inertial.calibration.bias;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Angle;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Approximately estimates accelerometer and gyroscope biases and noise PSD's
 * by averaging all provided samples when body position and orientation is known
 * while assuming that any cross coupling errors can be neglected.
 * <p>
 * This estimator must be used when the body where the accelerometer and gyroscope
 * is attached remains static on the same position with zero velocity and no rotation
 * speed while capturing data.
 * <p>
 * To compute PSD's this estimator assumes that accelerometer samples are obtained
 * at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, accelerometer and gyroscope sampling rate average can be
 * estimated using {@link com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator}.
 * <p>
 * Notice that in order to compute accelerometer and gyroscope biases, body position
 * and orientation must be known to account for gravity and Earth rotation effects.
 * <p>
 * Even though this estimator obtains approximate bias values, the obtained
 * result can be used to initialize some non-linear calibrators to obtain
 * more accurate results. Such calibrators are:
 * - com.irurueta.navigation.inertial.calibration.accelerometer.KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
 * - com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator
 * - com.irurueta.navigation.inertial.calibration.accelerometer.KnownPositionAccelerometerCalibrator}
 * - com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownFrameAccelerometerCalibrator and
 * any of its subclasses.
 * - com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownGravityNormAccelerometerCalibrator
 * and any of its subclasses.
 * - com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownPositionAccelerometerCalibrator and
 * any of its subclasses.
 * - com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator
 * - com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator
 * - com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasEasyGyroscopeCalibrator
 * - com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasTurntableGyroscopeCalibrator
 * - com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownBiasAndFrameGyroscopeCalibrator and any
 * of its subclasses.
 * - com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownBiasEasyGyroscopeCalibrator and any of
 * its subclasses.
 * - com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownBiasTurntableGyroscopeCalibrator and any
 * of its subclasses.
 * <p>
 * Even though this estimator can compute noise PSD's, if only noise PSD's levels
 * are required, estimators in {@link com.irurueta.navigation.inertial.calibration.noise} package should
 * be used instead.
 * <p>
 * This estimator does NOT compute average bias values over a period of time, it only
 * computes accumulated averages.
 */
public class BodyKinematicsBiasEstimator {

    /**
     * Default time interval between kinematics samples expressed in seconds (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Time interval expressed in seconds (s) between body kinematics samples.
     */
    private double timeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Contains body position, velocity (which will always be zero) and orientation
     * resolved around ECEF axes.
     * By default it is assumed that body is located at zero NED coordinates (latitude,
     * longitude and height) and with zero Euler angles representing rotation (roll = 0,
     * pith = 0, yaw = 0), which for Android devices it means that the device is flat
     * on a horizontal surface with the screen facing down.
     */
    private final ECEFFrame frame;

    /**
     * Listener to handle events raised by this estimator.
     */
    private BodyKinematicsBiasEstimatorListener listener;

    /**
     * Last provided body kinematics values.
     */
    private BodyKinematics lastBodyKinematics;

    /**
     * Contains estimated bias of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double biasFx;

    /**
     * Contains estimated bias of y coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double biasFy;

    /**
     * Contains estimated bias of z coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     */
    private double biasFz;

    /**
     * Contains estimated bias of x coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double biasAngularRateX;

    /**
     * Contains estimated bias of y coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double biasAngularRateY;

    /**
     * Contains estimated bias of z coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     */
    private double biasAngularRateZ;

    /**
     * Contains estimated variance of x coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     */
    private double varianceFx;

    /**
     * Contains estimated variance of y coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     */
    private double varianceFy;

    /**
     * Contains estimated variance of z coordinate of accelerometer sensed specific
     * force expressed in (m^2/s4).
     */
    private double varianceFz;

    /**
     * Contains estimated variance of x coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     */
    private double varianceAngularRateX;

    /**
     * Contains estimated variance of y coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     */
    private double varianceAngularRateY;

    /**
     * Contains estimated variance of z coordinate of gyroscope sensed angular rate
     * expressed in (rad^2/s^2).
     */
    private double varianceAngularRateZ;

    /**
     * Number of processed body kinematics samples.
     */
    private int numberOfProcessedSamples;

    /**
     * Number of processed body kinematics samples plus one.
     */
    private int numberOfProcessedSamplesPlusOne = 1;

    /**
     * Indicates that estimator is running.
     */
    private boolean running;

    /**
     * Theoretical expected body kinematics for provided body position and orientation,
     * and provided time interval, assuming that body remains at the same position
     * (zero velocity).
     * When body remains static, sensed specific force and angular rates will remain
     * constant due to gravity and Earth rotation.
     */
    private BodyKinematics expectedKinematics;

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     */
    public BodyKinematicsBiasEstimator() {
        frame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC coordinate transformation from body to local navigation
     *             (NED) coordinates. This contains orientation respect the horizon
     *             at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public BodyKinematicsBiasEstimator(final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException {
        frame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame(nedC));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     */
    public BodyKinematicsBiasEstimator(final double latitude, final double longitude, final double height) {
        frame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     */
    public BodyKinematicsBiasEstimator(final Angle latitude, final Angle longitude, final double height) {
        frame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     */
    public BodyKinematicsBiasEstimator(final Angle latitude, final Angle longitude, final Distance height) {
        frame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame(latitude, longitude, height));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public BodyKinematicsBiasEstimator(final NEDPosition position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException {
        frame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame(position, nedC));
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public BodyKinematicsBiasEstimator(final ECEFPosition position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException {
        frame = new ECEFFrame(position);
        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(frame);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public BodyKinematicsBiasEstimator(final BodyKinematicsBiasEstimatorListener listener) {
        this();
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public BodyKinematicsBiasEstimator(
            final CoordinateTransformation nedC, final BodyKinematicsBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param listener  listener to handle events raised by this estimator.
     */
    public BodyKinematicsBiasEstimator(
            final double latitude, final double longitude, final double height,
            final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param listener  listener to handle events raised by this estimator.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param listener  listener to handle events raised by this estimator.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public BodyKinematicsBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final BodyKinematicsBiasEstimatorListener listener) throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     */
    public BodyKinematicsBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final BodyKinematicsBiasEstimatorListener listener) throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(final double timeInterval) {
        this();
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final CoordinateTransformation nedC, final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final double latitude, final double longitude, final double height, final double timeInterval) {
        this(latitude, longitude, height);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final double height, final double timeInterval) {
        this(latitude, longitude, height);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height, final double timeInterval) {
        this(latitude, longitude, height);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC, final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC, final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC);
        try {
            setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(final double timeInterval, final BodyKinematicsBiasEstimatorListener listener) {
        this(timeInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final CoordinateTransformation nedC, final double timeInterval,
            final BodyKinematicsBiasEstimatorListener listener) throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, timeInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final double latitude, final double longitude, final double height,
            final double timeInterval, final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height, timeInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final double timeInterval, final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height, timeInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final double timeInterval, final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height, timeInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final double timeInterval, final BodyKinematicsBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, timeInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples
     *                     expressed in seconds (s).
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final double timeInterval, final BodyKinematicsBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, timeInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(final Time timeInterval) {
        this(convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(final CoordinateTransformation nedC, final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final double latitude, final double longitude, final double height, final Time timeInterval) {
        this(latitude, longitude, height, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final double height, final Time timeInterval) {
        this(latitude, longitude, height, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height, final Time timeInterval) {
        this(latitude, longitude, height, convertTime(timeInterval));
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC, final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, convertTime(timeInterval));
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC, final Time timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, convertTime(timeInterval));
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(final Time timeInterval, final BodyKinematicsBiasEstimatorListener listener) {
        this(convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final CoordinateTransformation nedC, final Time timeInterval,
            final BodyKinematicsBiasEstimatorListener listener) throws InvalidSourceAndDestinationFrameTypeException {
        this(nedC, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude expressed in radians (rad).
     * @param longitude    longitude expressed in radians (rad).
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final double latitude, final double longitude, final double height,
            final Time timeInterval, final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height expressed in meters (m).
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final double height, final Time timeInterval,
            final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude     latitude.
     * @param longitude    longitude.
     * @param height       height.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematicsBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final Time timeInterval, final BodyKinematicsBiasEstimatorListener listener) {
        this(latitude, longitude, height, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in NED coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final Time timeInterval, final BodyKinematicsBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, convertTime(timeInterval), listener);
    }

    /**
     * Constructor.
     *
     * @param position     body position expressed in ECEF coordinates.
     * @param nedC         coordinate transformation from body to local navigation
     *                     (NED) coordinates. This contains orientation respect the
     *                     horizon at current body location.
     * @param timeInterval time interval between body kinematics
     *                     (IMU acceleration + gyroscope) samples.
     * @param listener     listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IllegalArgumentException                      if provided time interval
     *                                                       is negative.
     */
    public BodyKinematicsBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final Time timeInterval, final BodyKinematicsBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, nedC, convertTime(timeInterval), listener);
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples expressed in seconds (s).
     *
     * @return time interval between body kinematics samples.
     */
    public double getTimeInterval() {
        return timeInterval;
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples expressed in seconds (s).
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        this.timeInterval = timeInterval;

        rebuildExpectedKinematics();
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @return time interval between body kinematics samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(timeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(timeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(convertTime(timeInterval));
    }

    /**
     * Gets current body position expressed in ECEF coordinates.
     *
     * @return current body position expressed in ECEF coordinates.
     */
    public ECEFPosition getEcefPosition() {
        return frame.getECEFPosition();
    }

    /**
     * Gets current body position expressed in ECEF coordinates.
     *
     * @param result instance where current body position will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        frame.getECEFPosition(result);
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param position current body position to be set.
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final ECEFPosition position) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPosition(position);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param x x position resolved around ECEF axes and expressed in meters (m).
     * @param y y position resolved around ECEF axes and expressed in meters (m).
     * @param z z position resolved around ECEF axes and expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final double x, final double y, final double z) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setCoordinates(x, y, z);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param x x position resolved around ECEF axes.
     * @param y y position resolved around ECEF axes.
     * @param z z position resolved around ECEF axes.
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final Distance x, final Distance y, final Distance z) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPositionCoordinates(x, y, z);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final Point3D position) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPosition(position);
        rebuildExpectedKinematics();
    }

    /**
     * Gets ECEF frame containing current body position and orientation expressed
     * in ECEF coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @return ECEF frame containing current body position and orientation resolved
     * around ECEF axes.
     */
    public ECEFFrame getEcefFrame() {
        return new ECEFFrame(frame);
    }

    /**
     * Gets ECEF frame containing current body position and orientation expressed
     * in ECEF coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @param result instance where ECEF frame containing current body position and
     *               orientation resolved around ECEF axes will be stored.
     */
    public void getEcefFrame(final ECEFFrame result) {
        frame.copyTo(result);
    }

    /**
     * Gets NED frame containing current body position and orientation expressed
     * in NED coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @return NED frame containing current body position and orientation resolved
     * around NED axes.
     */
    public NEDFrame getNedFrame() {
        return ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(frame);
    }

    /**
     * Gets NED frame containing current body position and orientation expressed
     * in NED coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @param result instance where NED frame containing current body position and
     *               orientation resolved around NED axes will be stored.
     */
    public void getNedFrame(final NEDFrame result) {
        ECEFtoNEDFrameConverter.convertECEFtoNED(frame, result);
    }

    /**
     * Gets current body position expressed in NED coordinates.
     *
     * @return current body position expressed in NED coordinates.
     */
    public NEDPosition getNedPosition() {
        return getNedFrame().getPosition();
    }

    /**
     * Gets current body position expressed in NED coordinates.
     *
     * @param result instance where current body position will be stored.
     */
    public void getNedPosition(final NEDPosition result) {
        getNedFrame().getPosition(result);
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param position current body position to be set.
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(final NEDPosition position) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(position);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate expressed in radians (rad).
     * @param longitude longitude NED coordinate expressed in radians (rad).
     * @param height    height NED coordinate expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final double latitude, final double longitude, final double height) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate.
     * @param longitude longitude NED coordinate.
     * @param height    height NED coordinate expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final Angle latitude, final Angle longitude, final double height) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate.
     * @param longitude longitude NED coordinate.
     * @param height    height NED coordinate.
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final Angle latitude, final Angle longitude, final Distance height) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Gets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that returned orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizon at current body position.
     * Typically it is more convenient to use {@link #getNedC()} to obtain orientation
     * relative to the ground or horizon at current body position. For instance, on
     * Android devices a NED orientation with Euler angles (roll = 0, pitch = 0,
     * yaw = 0) means that the device is laying flat on a horizontal surface with the
     * screen facing down towards the ground.
     *
     * @return current body orientation resolved on ECEF axes.
     */
    public CoordinateTransformation getEcefC() {
        return frame.getCoordinateTransformation();
    }

    /**
     * Gets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that returned orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizon at current body position.
     * Typically it is more convenient to use {@link #getNedC()} to obtain orientation
     * relative to the ground or horizon at current body position. For instance, on
     * Android devices a NED orientation with Euler angles (roll = 0, pitch = 0,
     * yaw = 0) means that the device is laying flat on a horizontal surface with the
     * screen facing down towards the ground.
     *
     * @param result instance where current body orientation resolved on ECEF axes
     *               will be stored.
     */
    public void getEcefC(final CoordinateTransformation result) {
        frame.getCoordinateTransformation(result);
    }

    /**
     * Sets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that ECEF orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizon at current body position.
     * Typically it is more convenient to use
     * {@link #setNedC(CoordinateTransformation)} to specify orientation relative to
     * the ground or horizon at current body position.
     * For instance, on Android devices a NED orientation with Euler angles (roll = 0,
     * pitch = 0, yaw = 0) means that the device is laying flat on a horizontal surface
     * with the screen facing down towards the ground.
     *
     * @param ecefC body orientation resolved on ECEF axes to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     */
    public void setEcefC(final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Gets current body orientation as a transformation from body to NED coordinates.
     * Notice that returned orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @return current body orientation resolved on NED axes.
     */
    public CoordinateTransformation getNedC() {
        return getNedFrame().getCoordinateTransformation();
    }

    /**
     * Gets current body orientation as a transformation from body to NED coordinates.
     * Notice that returned orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @param result instance where current body orientation resolved on NED axes
     *               will be stored.
     */
    public void getNedC(final CoordinateTransformation result) {
        getNedFrame().getCoordinateTransformation(result);
    }

    /**
     * Sets current body orientation as a transformation from body to NED coordinates.
     * Notice that provided orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @param nedC orientation resolved on NED axes to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     */
    public void setNedC(final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(NEDPosition)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final NEDPosition nedPosition, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(nedPosition);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(double, double, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final double latitude, final double longitude, final double height,
            final CoordinateTransformation nedC) throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final Angle latitude, final Angle longitude, final double height,
            final CoordinateTransformation nedC) throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, Distance)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final Angle latitude, final Angle longitude, final Distance height,
            final CoordinateTransformation nedC) throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param ecefC        body to ECEF coordinate transformation indicating body
     *                     orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(ECEFPosition)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final ECEFPosition ecefPosition, final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPosition(ecefPosition);
        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param x     x coordinate of ECEF position expressed in meters (m).
     * @param y     y coordinate of ECEF position expressed in meters (m).
     * @param z     z coordinate of ECEF position expressed in meters (m).
     * @param ecefC body to ECEF coordinate transformation indicating body
     *              orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(double, double, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final double x, final double y, final double z,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setCoordinates(x, y, z);
        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param x     x coordinate of ECEF position.
     * @param y     y coordinate of ECEF position.
     * @param z     z coordinate of ECEF position.
     * @param ecefC body to ECEF coordinate transformation indicating body
     *              orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Distance, Distance, Distance)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final Distance x, final Distance y, final Distance z,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPositionCoordinates(x, y, z);
        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @param ecefC    body to ECEF coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Point3D)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final Point3D position, final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPosition(position);
        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param position position expressed on NED coordinates.
     * @param ecefC    body to ECEF coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(NEDPosition)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final NEDPosition position,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(position);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(double, double, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final double latitude, final double longitude, final double height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final Angle latitude, final Angle longitude, final double height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, Distance)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final Angle latitude, final Angle longitude, final Distance height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        final var nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        frame.setCoordinateTransformation(ecefC);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(ECEFPosition)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final ECEFPosition ecefPosition, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPosition(ecefPosition);

        final var nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param x    x coordinate of ECEF position expressed in meters (m).
     * @param y    y coordinate of ECEF position expressed in meters (m).
     * @param z    z coordinate of ECEF position expressed in meters (m).
     * @param nedC body to NED coordinate transformation indicating body
     *             orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(double, double, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final double x, final double y, final double z, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setCoordinates(x, y, z);

        final var nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param x    x coordinate of ECEF position.
     * @param y    y coordinate of ECEF position.
     * @param z    z coordinate of ECEF position.
     * @param nedC body to NED coordinate transformation indicating body
     *             orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Distance, Distance, Distance)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final Distance x, final Distance y, final Distance z, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPositionCoordinates(x, y, z);

        final var nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @param nedC     body to NED coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Point3D)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final Point3D position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        frame.setPosition(position);

        final var nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
        rebuildExpectedKinematics();
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public BodyKinematicsBiasEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final BodyKinematicsBiasEstimatorListener listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets last provided body kinematics values or null if not available.
     *
     * @return last provided body kinematics values or null.
     */
    public BodyKinematics getLastBodyKinematics() {
        return lastBodyKinematics != null ? new BodyKinematics(lastBodyKinematics) : null;
    }

    /**
     * Gets last provided body kinematics values.
     *
     * @param result instance where last provided body kinematics will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getLastBodyKinematics(final BodyKinematics result) {
        if (lastBodyKinematics != null) {
            lastBodyKinematics.copyTo(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated bias of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     *
     * @return bias of x coordinate of sensed specific force.
     */
    public double getBiasFx() {
        return biasFx;
    }

    /**
     * Gets estimated bias of x coordinate of accelerometer sensed specific force.
     *
     * @return bias of x coordinate of sensed specific force.
     */
    public Acceleration getBiasFxAsAcceleration() {
        return new Acceleration(biasFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of x coordinate of accelerometer sensed specific force.
     *
     * @param result instance where bias of x coordinate of sensed specific force
     *               will be stored.
     */
    public void getBiasFxAsAcceleration(final Acceleration result) {
        result.setValue(biasFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of x coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     *
     * @return bias of y coordinate of sensed specific force.
     */
    public double getBiasFy() {
        return biasFy;
    }

    /**
     * Gets estimated bias of y coordinate of accelerometer sensed specific force.
     *
     * @return bias of y coordinate of sensed specific force.
     */
    public Acceleration getBiasFyAsAcceleration() {
        return new Acceleration(biasFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of y coordinate of accelerometer sensed specific force.
     *
     * @param result instance where bias of y coordinate of sensed specific force
     *               will be stored.
     */
    public void getBiasFyAsAcceleration(final Acceleration result) {
        result.setValue(biasFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of z coordinate of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2).
     *
     * @return bias of z coordinate of sensed specific force.
     */
    public double getBiasFz() {
        return biasFz;
    }

    /**
     * Gets estimated bias of z coordinate of accelerometer sensed specific force.
     *
     * @return bias of z coordinate of sensed specific force.
     */
    public Acceleration getBiasFzAsAcceleration() {
        return new Acceleration(biasFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of z coordinate of accelerometer sensed specific force.
     *
     * @param result instance where bias of z coordinate of sensed specific force
     *               will be stored.
     */
    public void getBiasFzAsAcceleration(final Acceleration result) {
        result.setValue(biasFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of x coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     *
     * @return bias of x coordinate of sensed angular rate.
     */
    public double getBiasAngularRateX() {
        return biasAngularRateX;
    }

    /**
     * Gets estimated bias of x coordinate of gyroscope sensed angular rate.
     *
     * @return bias of x coordinate of sensed angular rate.
     */
    public AngularSpeed getBiasAngularRateXAsAngularSpeed() {
        return new AngularSpeed(biasAngularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of x coordinate of gyroscope sensed angular rate.
     *
     * @param result instance where bias of x coordinate of sensed angular rate
     *               will be stored.
     */
    public void getBiasAngularRateXAsAngularSpeed(final AngularSpeed result) {
        result.setValue(biasAngularRateX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of y coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     *
     * @return bias of y coordinate of sensed angular rate.
     */
    public double getBiasAngularRateY() {
        return biasAngularRateY;
    }

    /**
     * Gets estimated bias of y coordinate of gyroscope sensed angular rate.
     *
     * @return bias of y coordinate of sensed angular rate.
     */
    public AngularSpeed getBiasAngularRateYAsAngularSpeed() {
        return new AngularSpeed(biasAngularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of y coordinate of gyroscope sensed angular rate.
     *
     * @param result instance where bias of y coordinate of sensed angular rate
     *               will be stored.
     */
    public void getBiasAngularRateYAsAngularSpeed(final AngularSpeed result) {
        result.setValue(biasAngularRateY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of z coordinate of gyroscope sensed angular rate
     * expressed in radians per second (rad/s).
     *
     * @return bias of z coordinate of sensed angular rate.
     */
    public double getBiasAngularRateZ() {
        return biasAngularRateZ;
    }

    /**
     * Gets estimated bias of z coordinate of gyroscope sensed angular rate.
     *
     * @return bias of z coordinate of sensed angular rate.
     */
    public AngularSpeed getBiasAngularRateZAsAngularSpeed() {
        return new AngularSpeed(biasAngularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of z coordinate of gyroscope sensed angular rate.
     *
     * @param result instance where bias of z coordinate of sensed angular rate
     *               will be stored.
     */
    public void getBiasAngularRateZAsAngularSpeed(final AngularSpeed result) {
        result.setValue(biasAngularRateZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated bias of accelerometer sensed specific force.
     *
     * @return estimated bias of accelerometer sensed specific force.
     */
    public AccelerationTriad getBiasF() {
        return new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasFx, biasFy, biasFz);
    }

    /**
     * Gets estimated bias of accelerometer sensed specific force.
     *
     * @param result instance where bias of sensed specific force will
     *               be stored.
     */
    public void getBiasF(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(biasFx, biasFy, biasFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated bias of gyroscope sensed angular rate.
     *
     * @return estimated bias of gyroscope sensed angular rate.
     */
    public AngularSpeedTriad getBiasAngularRate() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                biasAngularRateX, biasAngularRateY, biasAngularRateZ);
    }

    /**
     * Gets estimated bias of gyroscope sensed angular rate.
     *
     * @param result instance where bias of gyroscope sensed angular
     *               rate will be stored.
     */
    public void getBiasAngularRate(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(biasAngularRateX, biasAngularRateY, biasAngularRateZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets body kinematics containing estimated bias values for accelerometer
     * and gyroscope.
     *
     * @return body kinematics containing estimated bias values.
     */
    public BodyKinematics getBiasesAsBodyKinematics() {
        final var result = new BodyKinematics();
        getBiasesAsBodyKinematics(result);
        return result;
    }

    /**
     * Gets body kinematics containing estimated bias values for accelerometer
     * and gyroscope.
     *
     * @param result instance where body kinematics containing estimated bias
     *               values will be stored.
     */
    public void getBiasesAsBodyKinematics(final BodyKinematics result) {
        result.setSpecificForceCoordinates(biasFx, biasFy, biasFz);
        result.setAngularRateCoordinates(biasAngularRateX, biasAngularRateY, biasAngularRateZ);
    }

    /**
     * Gets estimated variance of x coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of x coordinate of sensed specific force.
     */
    public double getVarianceFx() {
        return varianceFx;
    }

    /**
     * Gets estimated variance of y coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of y coordinate of sensed specific force.
     */
    public double getVarianceFy() {
        return varianceFy;
    }

    /**
     * Gets estimated variance of z coordinate of accelerometer sensed specific
     * force expressed in (m^2/s^4).
     *
     * @return estimated variance of z coordinate of sensed specific force.
     */
    public double getVarianceFz() {
        return varianceFz;
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
     * sensed specific force expressed in (m/s^2).
     *
     * @return estimated standard deviation of x coordinate of sensed specific force.
     */
    public double getStandardDeviationFx() {
        return Math.sqrt(varianceFx);
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of x coordinate of sensed specific force.
     */
    public Acceleration getStandardDeviationFxAsAcceleration() {
        return new Acceleration(getStandardDeviationFx(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of x coordinate
     *               of sensed specific force will be stored.
     */
    public void getStandardDeviationFxAsAcceleration(final Acceleration result) {
        result.setValue(getStandardDeviationFx());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force expressed in (m/s^2).
     *
     * @return estimated standard deviation of y coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationFy() {
        return Math.sqrt(varianceFy);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of y coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationFyAsAcceleration() {
        return new Acceleration(getStandardDeviationFy(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of y coordinate
     *               of sensed specific force will be stored.
     */
    public void getStandardDeviationFyAsAcceleration(final Acceleration result) {
        result.setValue(getStandardDeviationFy());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force expressed in (m/s^2).
     *
     * @return estimated standard deviation of z coordinate of sensed specific
     * force.
     */
    public double getStandardDeviationFz() {
        return Math.sqrt(varianceFz);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force.
     *
     * @return estimated standard deviation of z coordinate of sensed specific
     * force.
     */
    public Acceleration getStandardDeviationFzAsAcceleration() {
        return new Acceleration(getStandardDeviationFz(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of accelerometer
     * sensed specific force.
     *
     * @param result instance where estimated standard deviation of z coordinate
     *               of sensed specific force will be stored.
     */
    public void getStandardDeviationFzAsAcceleration(final Acceleration result) {
        result.setValue(getStandardDeviationFz());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of accelerometer sensed
     * specific force.
     *
     * @return estimated standard deviation of accelerometer
     */
    public AccelerationTriad getStandardDeviationF() {
        return new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                getStandardDeviationFx(),
                getStandardDeviationFy(),
                getStandardDeviationFz());
    }

    /**
     * Gets estimated standard deviation of accelerometer sensed
     * specific force.
     *
     * @param result instance where estimated standard deviation of
     *               accelerometer will be stored.
     */
    public void getStandardDeviationF(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(getStandardDeviationFx(), getStandardDeviationFy(), getStandardDeviationFz(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets average of estimated standard deviation of accelerometer sensed specific
     * force for all coordinates expressed in meters per squared second (m/s^2).
     *
     * @return average of estimated standard deviation of accelerometer.
     */
    public double getAverageAccelerometerStandardDeviation() {
        return (getStandardDeviationFx() + getStandardDeviationFy() + getStandardDeviationFz()) / 3.0;
    }

    /**
     * Gets average of estimated standard deviation of accelerometer sensed specific
     * force for all coordinates.
     *
     * @return average of estimated standard deviation of accelerometer.
     */
    public Acceleration getAverageAccelerometerStandardDeviationAsAcceleration() {
        return new Acceleration(getAverageAccelerometerStandardDeviation(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets average of estimated standard deviation of accelerometer sensed specific
     * force for all coordinates.
     *
     * @param result instance where result data will be copied to.
     */
    public void getAverageAccelerometerStandardDeviationAsAcceleration(final Acceleration result) {
        result.setValue(getAverageAccelerometerStandardDeviation());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope sensed angular
     * rate expressed in (rad/s).
     *
     * @return estimated standard deviation of x coordinate of sensed angular rate.
     */
    public double getStandardDeviationAngularRateX() {
        return Math.sqrt(varianceAngularRateX);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope sensed angular
     * rate.
     *
     * @return estimated standard deviation of x coordinate of sensed angular rate.
     */
    public AngularSpeed getStandardDeviationAngularRateXAsAngularSpeed() {
        return new AngularSpeed(getStandardDeviationAngularRateX(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of x coordinate of gyroscope sensed angular
     * rate.
     *
     * @param result instance where estimated standard deviation of x coordinate of
     *               sensed angular rate will be stored.
     */
    public void getStandardDeviationAngularRateXAsAngularSpeed(final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateX());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope sensed angular
     * rate expressed in (rad/s).
     *
     * @return estimated standard deviation of y coordinate of sensed angular rate.
     */
    public double getStandardDeviationAngularRateY() {
        return Math.sqrt(varianceAngularRateY);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope sensed angular
     * rate.
     *
     * @return estimated standard deviation of y coordinate of sensed angular rate.
     */
    public AngularSpeed getStandardDeviationAngularRateYAsAngularSpeed() {
        return new AngularSpeed(getStandardDeviationAngularRateY(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of y coordinate of gyroscope sensed angular
     * rate.
     *
     * @param result instance where estimated standard deviation of y coordinate of
     *               sensed angular rate will be stored.
     */
    public void getStandardDeviationAngularRateYAsAngularSpeed(final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateY());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope sensed angular
     * rate expressed in (rad/s).
     *
     * @return estimated standard deviation of z coordinate of sensed angular rate.
     */
    public double getStandardDeviationAngularRateZ() {
        return Math.sqrt(varianceAngularRateZ);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope sensed angular
     * rate.
     *
     * @return estimated standard deviation of z coordinate of sensed angular rate.
     */
    public AngularSpeed getStandardDeviationAngularRateZAsAngularSpeed() {
        return new AngularSpeed(getStandardDeviationAngularRateZ(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of z coordinate of gyroscope sensed angular
     * rate.
     *
     * @param result instance where estimated standard deviation of z coordinate of
     *               sensed angular rate will be stored.
     */
    public void getStandardDeviationAngularRateZAsAngularSpeed(final AngularSpeed result) {
        result.setValue(getStandardDeviationAngularRateZ());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviation of sensed angular rate.
     *
     * @return estimated standard deviation of sensed angular rate.
     */
    public AngularSpeedTriad getStandardDeviationAngularRate() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                getStandardDeviationAngularRateX(),
                getStandardDeviationAngularRateY(),
                getStandardDeviationAngularRateZ());
    }

    /**
     * Gets estimated standard deviation of sensed angular rate.
     *
     * @param result instance where estimated standard deviation of
     *               sensed angular rate.
     */
    public void getStandardDeviationAngularRate(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(getStandardDeviationAngularRateX(),
                getStandardDeviationAngularRateY(),
                getStandardDeviationAngularRateZ(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets average of estimated standard deviation of gyroscope sensed angular rate
     * for all coordinates expressed in radians per second (rad/s).
     *
     * @return average of estimated standard deviation of gyroscope.
     */
    public double getAverageGyroscopeStandardDeviation() {
        return (getStandardDeviationAngularRateX() + getStandardDeviationAngularRateY()
                + getStandardDeviationAngularRateZ()) / 3.0;
    }

    /**
     * Gets average of estimated standard deviation of gyroscope sensed angular rate
     * for all coordinates.
     *
     * @return average of estimated standard deviation of gyroscope.
     */
    public AngularSpeed getAverageGyroscopeStandardDeviationAsAngularSpeed() {
        return new AngularSpeed(getAverageGyroscopeStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets average of estimated standard deviation of gyroscope sensed angular rate
     * for all coordinates.
     *
     * @param result instance where result data will be copied to.
     */
    public void getAverageGyroscopeStandardDeviationAsAngularSpeed(final AngularSpeed result) {
        result.setValue(getAverageGyroscopeStandardDeviation());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated standard deviations of accelerometer and gyroscope components
     * as a body kinematics instance.
     *
     * @return a body kinematics instance containing standard deviation values.
     */
    public BodyKinematics getStandardDeviationsAsBodyKinematics() {
        return new BodyKinematics(getStandardDeviationFx(),
                getStandardDeviationFy(),
                getStandardDeviationFz(),
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
    public void getStandardDeviationsAsBodyKinematics(final BodyKinematics result) {
        result.setSpecificForceCoordinates(getStandardDeviationFx(),
                getStandardDeviationFy(), getStandardDeviationFz());
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
    public double getPSDFx() {
        return varianceFx * timeInterval;
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on y axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on y axis.
     */
    public double getPSDFy() {
        return varianceFy * timeInterval;
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) on z axis expressed
     * in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD on z axis.
     */
    public double getPSDFz() {
        return varianceFz * timeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on x axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on x axis.
     */
    public double getPSDAngularRateX() {
        return varianceAngularRateX * timeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on y axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on y axis.
     */
    public double getPSDAngularRateY() {
        return varianceAngularRateY * timeInterval;
    }

    /**
     * Gets gyroscope noise PSD (Power Spectral Density) on z axis expressed
     * in (rad^2/s).
     *
     * @return gyroscope noise PSD on z axis.
     */
    public double getPSDAngularRateZ() {
        return varianceAngularRateZ * timeInterval;
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on x axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on x axis.
     */
    public double getRootPSDFx() {
        return Math.sqrt(getPSDFx());
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on y axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on y axis.
     */
    public double getRootPSDFy() {
        return Math.sqrt(getPSDFy());
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) on z axis
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer noise root PSD on z axis.
     */
    public double getRootPSDFz() {
        return Math.sqrt(getPSDFz());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on x axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on x axis.
     */
    public double getRootPSDAngularRateX() {
        return Math.sqrt(getPSDAngularRateX());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on y axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on y axis.
     */
    public double getRootPSDAngularRateY() {
        return Math.sqrt(getPSDAngularRateY());
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) on z axis
     * expressed in (rad * s^-0.5).
     *
     * @return gyroscope noise root PSD on z axis.
     */
    public double getRootPSDAngularRateZ() {
        return Math.sqrt(getPSDAngularRateZ());
    }

    /**
     * Gets average accelerometer noise PSD (Power Spectral Density) among
     * x,y,z components expressed as (m^2/s^3).
     *
     * @return average accelerometer noise PSD.
     */
    public double getAccelerometerNoisePSD() {
        return (getPSDFx() + getPSDFy() + getPSDFz()) / 3.0;
    }

    /**
     * Gets accelerometer noise root PSD (Power Spectral Density) which is the
     * norm of root PSD components expressed as (m * s^-1.5).
     *
     * @return average accelerometer noise root PSD.
     */
    public double getAccelerometerNoiseRootPSD() {
        return Math.sqrt(getPSDFx() + getPSDFy() + getPSDFz());
    }

    /**
     * Gets average gyroscope noise PSD (Power Spectral Density) among
     * x,y,z components expressed in (rad^2/s).
     *
     * @return average gyroscope noise PSD.
     */
    public double getGyroNoisePSD() {
        return (getPSDAngularRateX() + getPSDAngularRateY() + getPSDAngularRateZ()) / 3.0;
    }

    /**
     * Gets gyroscope noise root PSD (Power Spectral Density) which is the
     * norm of root PSD components expressed in (rad * s^-0.5).
     *
     * @return average gyroscope noise root PSD.
     */
    public double getGyroNoiseRootPSD() {
        return Math.sqrt(getPSDAngularRateX() + getPSDAngularRateY() + getPSDAngularRateZ());
    }

    /**
     * Gets estimated bias of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2) as a 3x1 matrix column vector.
     *
     * @return estimated bias of accelerometer sensed specific force.
     */
    public Matrix getAccelerometerBias() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getAccelerometerBias(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }

        return result;
    }

    /**
     * Gets estimated bias of accelerometer sensed specific force
     * expressed in meters per squared second (m/s^2) as a 3x1 matrix column vector.
     *
     * @param result instance where data will be copied to. Must be 3x1.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void getAccelerometerBias(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        result.setElementAtIndex(0, biasFx);
        result.setElementAtIndex(1, biasFy);
        result.setElementAtIndex(2, biasFz);
    }

    /**
     * Gets estimated bias of gyroscope sensed angular rates
     * expressed in radians per second (rad/s) as a 3x1 matrix column vector.
     *
     * @return estimated bias of gyroscope sensed angular rates.
     */
    public Matrix getGyroBias() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getGyroBias(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }

        return result;
    }

    /**
     * Gets estimated bias of gyroscope sensed angular rates
     * expressed in radians per second (rad/s) as a 3x1 matrix column vector.
     *
     * @param result instance where data will be copied to. Must be 3x1.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void getGyroBias(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        result.setElementAtIndex(0, biasAngularRateX);
        result.setElementAtIndex(1, biasAngularRateY);
        result.setElementAtIndex(2, biasAngularRateZ);
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
     * Gets amount of total elapsed time since first processed measurement expressed
     * in seconds (s).
     *
     * @return amount of total elapsed time.
     */
    public double getElapsedTimeSeconds() {
        return numberOfProcessedSamples * timeInterval;
    }

    /**
     * Gets amount of total elapsed time since first processed measurement.
     *
     * @return amount of total elapsed time.
     */
    public Time getElapsedTime() {
        return new Time(getElapsedTimeSeconds(), TimeUnit.SECOND);
    }

    /**
     * Gets amount of total elapsed time since first processed measurement.
     *
     * @param result instance where result will be stored.
     */
    public void getElapsedTime(final Time result) {
        result.setValue(getElapsedTimeSeconds());
        result.setUnit(TimeUnit.SECOND);
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
     * Gets theoretically expected body kinematics for provided body position and
     * orientation, and provided time interval, assuming that body remains at the
     * same position (zero velocity).
     * When body remains static, sensed specific force and angular rates will remain
     * constant due to gravity and Earth rotation.
     *
     * @return expected body kinematics.
     */
    public BodyKinematics getExpectedKinematics() {
        return new BodyKinematics(expectedKinematics);
    }

    /**
     * Gets theoretically expected body kinematics for provided body position and
     * orientation, and provided time interval, assuming that body remains at the
     * same position (zero velocity).
     * When body remains static, sensed specific force and angular rates will remain
     * constant due to gravity and Earth rotation.
     *
     * @param result instance where expected body kinematics will be stored.
     */
    public void getExpectedKinematics(final BodyKinematics result) {
        expectedKinematics.copyTo(result);
    }

    /**
     * Adds a sample of body kinematics (accelerometer + gyroscope readings) obtained
     * from an IMU.
     *
     * @param kinematics kinematics instance to be added and processed.
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyKinematics(final BodyKinematics kinematics) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        running = true;

        if (lastBodyKinematics == null && listener != null) {
            listener.onStart(this);
        }

        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();
        final var angularRateX = kinematics.getAngularRateX();
        final var angularRateY = kinematics.getAngularRateY();
        final var angularRateZ = kinematics.getAngularRateZ();

        final var expectedFx = expectedKinematics.getFx();
        final var expectedFy = expectedKinematics.getFy();
        final var expectedFz = expectedKinematics.getFz();
        final var expectedAngularRateX = expectedKinematics.getAngularRateX();
        final var expectedAngularRateY = expectedKinematics.getAngularRateY();
        final var expectedAngularRateZ = expectedKinematics.getAngularRateZ();

        final var diffFx = fx - expectedFx;
        final var diffFy = fy - expectedFy;
        final var diffFz = fz - expectedFz;
        final var diffAngularRateX = angularRateX - expectedAngularRateX;
        final var diffAngularRateY = angularRateY - expectedAngularRateY;
        final var diffAngularRateZ = angularRateZ - expectedAngularRateZ;

        // compute biases
        final var tmp = (double) numberOfProcessedSamples / (double) numberOfProcessedSamplesPlusOne;
        biasFx = biasFx * tmp + diffFx / numberOfProcessedSamplesPlusOne;
        biasFy = biasFy * tmp + diffFy / numberOfProcessedSamplesPlusOne;
        biasFz = biasFz * tmp + diffFz / numberOfProcessedSamplesPlusOne;

        biasAngularRateX = biasAngularRateX * tmp + diffAngularRateX / numberOfProcessedSamplesPlusOne;
        biasAngularRateY = biasAngularRateY * tmp + diffAngularRateY / numberOfProcessedSamplesPlusOne;
        biasAngularRateZ = biasAngularRateZ * tmp + diffAngularRateZ / numberOfProcessedSamplesPlusOne;

        // compute variances
        final var diffBiasFx = diffFx - biasFx;
        final var diffBiasFy = diffFy - biasFy;
        final var diffBiasFz = diffFz - biasFz;
        final var diffBiasAngularRateX = diffAngularRateX - biasAngularRateX;
        final var diffBiasAngularRateY = diffAngularRateY - biasAngularRateY;
        final var diffBiasAngularRateZ = diffAngularRateZ - biasAngularRateZ;

        final var diffBiasFx2 = diffBiasFx * diffBiasFx;
        final var diffBiasFy2 = diffBiasFy * diffBiasFy;
        final var diffBiasFz2 = diffBiasFz * diffBiasFz;
        final var diffBiasAngularRateX2 = diffBiasAngularRateX * diffBiasAngularRateX;
        final var diffBiasAngularRateY2 = diffBiasAngularRateY * diffBiasAngularRateY;
        final var diffBiasAngularRateZ2 = diffBiasAngularRateZ * diffBiasAngularRateZ;

        varianceFx = varianceFx * tmp + diffBiasFx2 / numberOfProcessedSamplesPlusOne;
        varianceFy = varianceFy * tmp + diffBiasFy2 / numberOfProcessedSamplesPlusOne;
        varianceFz = varianceFz * tmp + diffBiasFz2 / numberOfProcessedSamplesPlusOne;

        varianceAngularRateX = varianceAngularRateX * tmp + diffBiasAngularRateX2 / numberOfProcessedSamplesPlusOne;
        varianceAngularRateY = varianceAngularRateY * tmp + diffBiasAngularRateY2 / numberOfProcessedSamplesPlusOne;
        varianceAngularRateZ = varianceAngularRateZ * tmp + diffBiasAngularRateZ2 / numberOfProcessedSamplesPlusOne;

        lastBodyKinematics = kinematics;

        numberOfProcessedSamples++;
        numberOfProcessedSamplesPlusOne++;

        if (listener != null) {
            listener.onBodyKinematicsAdded(this);
        }

        running = false;
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
        lastBodyKinematics = null;
        biasFx = 0.0;
        biasFy = 0.0;
        biasFz = 0.0;
        biasAngularRateX = 0.0;
        biasAngularRateY = 0.0;
        biasAngularRateZ = 0.0;
        varianceFx = 0.0;
        varianceFy = 0.0;
        varianceFz = 0.0;
        varianceAngularRateX = 0.0;
        varianceAngularRateY = 0.0;
        varianceAngularRateZ = 0.0;
        numberOfProcessedSamples = 0;
        numberOfProcessedSamplesPlusOne = 1;

        if (listener != null) {
            listener.onReset(this);
        }

        running = false;

        return true;
    }

    /**
     * Converts provided time instance to seconds.
     *
     * @param time instance to be converted.
     * @return obtained conversion in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Rebuilds expected theoretical kinematics for provided body position
     * and orientation and provided time interval, assuming that body
     * remains at the same position (zero velocity).
     * When body remains static, sensed specific force and angular rates will remain
     * constant due to gravity and Earth rotation.
     */
    private void rebuildExpectedKinematics() {
        if (frame == null) {
            return;
        }
        if (expectedKinematics == null) {
            expectedKinematics = new BodyKinematics();
        }

        final var ecefC = getEcefC();
        final var x = frame.getX();
        final var y = frame.getY();
        final var z = frame.getZ();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                x, y, z, expectedKinematics);
    }
}
