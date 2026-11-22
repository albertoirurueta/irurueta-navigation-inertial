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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.gnss.GNSSBiasesGenerator;
import com.irurueta.navigation.gnss.GNSSConfig;
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.navigation.gnss.GNSSLeastSquaresPositionAndVelocityEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.navigation.gnss.GNSSMeasurementsGenerator;
import com.irurueta.navigation.gnss.SatelliteECEFPositionAndVelocityGenerator;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class INSGNSSTightlyCoupledKalmanFilteredEstimatorTest implements INSGNSSTightlyCoupledKalmanFilteredEstimatorListener {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double MIN_EPOCH_INTERVAL = 1e-5;
    private static final double MAX_EPOCH_INTERVAL = 1.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_TIME = 0.0;
    private static final double MAX_TIME = 1.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_USER_HEIGHT = -50.0;
    private static final double MAX_USER_HEIGHT = 50.0;

    private static final double MIN_USER_VELOCITY_VALUE = -2.0;
    private static final double MAX_USER_VELOCITY_VALUE = 2.0;

    private static final double MIN_DEGREES_PER_SECOND = -10.0;
    private static final double MAX_DEGREES_PER_SECOND = 10.0;

    /**
     * Yaw rate of the yawing trajectory, expressed in radians per second.
     */
    private static final double YAW_RATE = 0.1;

    /**
     * Duration of the yawing, northbound and error-free measurement trajectories, expressed in
     * seconds.
     */
    private static final double TRAJECTORY_DURATION = 10.0;

    /**
     * Yaw expected at the end of the yawing trajectory (rate times duration), expressed in radians.
     */
    private static final double EXPECTED_FINAL_YAW = YAW_RATE * TRAJECTORY_DURATION;

    /**
     * Speed of the northbound trajectory, expressed in meters per second.
     */
    private static final double NORTH_SPEED = 10.0;

    /**
     * Duration of each of the two halves (acceleration, deceleration) of the trajectory used to check
     * that every IMU sample within an epoch is integrated, expressed in seconds.
     */
    private static final double HALF_EPOCH_DURATION = 0.5;

    /**
     * Epoch interval, expressed in seconds, used to check that IMU samples arriving within a
     * Kalman epoch are integrated. It equals the whole duration of the trajectory.
     */
    private static final double ONE_SECOND_EPOCH_INTERVAL = 1.0;

    /**
     * Tolerance, in radians, for the yaw integrated over the yawing trajectory.
     * The kinematics generated by {@code ECEFKinematicsEstimator} are the exact inverse of the
     * attitude update of {@code ECEFInertialNavigator} up to rounding errors, which after 1000 samples
     * remain several orders of magnitude below this value; 1e-6 rad (0.2 arc-seconds) is far below
     * the 1 rad that is lost when rotation is not integrated.
     */
    private static final double YAW_TOLERANCE = 1e-6;

    /**
     * Minimum distance, in meters, for two solutions to be considered different: three orders of
     * magnitude above the tolerance of the chained navigator oracle, and well below the 0.5 m that
     * separate the per-sample integration from a single step using only the last sample.
     */
    private static final double DIFFERENT_POSITION_THRESHOLD = 1e-3;

    /**
     * Tolerance, in meters, when comparing against the chained navigator oracle. Both sides execute
     * the same floating point operations, so differences can only come from rounding of ECEF
     * coordinates of magnitude 6.4e6 m (ulp of 1e-9 m).
     */
    private static final double CHAIN_POSITION_TOLERANCE = 1e-6;

    /**
     * Tolerance, in meters per second, when comparing velocity against the truth or the chained
     * navigator oracle: the navigator velocity update is the exact inverse of the kinematics
     * estimation, so only rounding errors (below 1e-9 m/s) remain.
     */
    private static final double CHAIN_VELOCITY_TOLERANCE = 1e-6;

    /**
     * Tolerance for each element of the body to ECEF rotation matrix when comparing against the
     * chained navigator oracle: elements are bounded by one and both sides execute the same
     * operations, so differences are bounded by accumulated rounding (1e-13 after 1000 samples).
     */
    private static final double CHAIN_ROTATION_TOLERANCE = 1e-9;

    /**
     * Tolerance, in meters, for the position of a solution initialised from error-free GNSS
     * measurements against the truth. The least squares estimator and the Kalman epoch do not predict
     * pseudo-ranges with exactly the same approximations as the measurements generator, which leaves
     * a measured error of about 3 cm after the initialisation, plus the drift caused by the initial
     * velocity error (see {@link #GNSS_VELOCITY_TOLERANCE}) over at most 10 s; 0.1 m bounds both with
     * margin while being three orders of magnitude below the 100 m lost when translation is not
     * integrated.
     */
    private static final double GNSS_POSITION_TOLERANCE = 0.1;

    /**
     * Tolerance, in meters per second, for the velocity of a solution initialised from error-free GNSS
     * measurements against the truth: for the same reason as {@link #GNSS_POSITION_TOLERANCE} a
     * measured error of about 1.3 mm/s is left after the initialisation; 5e-3 m/s bounds it with
     * margin.
     */
    private static final double GNSS_VELOCITY_TOLERANCE = 5e-3;

    /**
     * Tolerance when comparing coordinate transformations that must be equal up to rounding: the
     * attitude is re-orthonormalised after each Kalman epoch, which only introduces rounding errors
     * (1e-15) on elements bounded by one.
     */
    private static final double ABSOLUTE_ERROR = 1e-8;

    /**
     * Factor of the displacement of a uniformly accelerated body (g * t^2 / 2), also used to obtain
     * a timestamp older than a given one.
     */
    private static final double HALF = 0.5;

    /**
     * Tolerance, in meters, for the displacement of a free fall of up to 1 s starting with a speed of
     * up to 3.5 m/s (2 m/s per axis) against v0 * t + g * t^2 / 2. The neglected Coriolis acceleration
     * is at most 2 * 7.29e-5 rad/s * (3.5 + 9.8) m/s = 1.9e-3 m/s^2, i.e. 1e-3 m after 1 s, and the
     * gravity gradient over the 8.4 m travelled adds 3e-6 s^-2 * 8.4 m = 2.5e-5 m/s^2; hence 1e-2 m.
     */
    private static final double FREE_FALL_POSITION_TOLERANCE = 1e-2;

    /**
     * Tolerance, in meters per second, for the velocity of a free fall of up to 1 s starting with a
     * speed of up to 3.5 m/s against v0 + g * t: the neglected Coriolis acceleration (1.9e-3 m/s^2 at
     * most, see {@link #FREE_FALL_POSITION_TOLERANCE}) integrates to 1.9e-3 m/s; hence 5e-3 m/s.
     */
    private static final double FREE_FALL_VELOCITY_TOLERANCE = 5e-3;

    /**
     * Tolerance, in radians, for the angle rotated by the body with respect to the ECEF frame during
     * up to 1 s against the measured angular rate times the elapsed time: the gyroscope measures the
     * rotation with respect to the inertial frame, which differs by the Earth rotation (7.29e-5 rad/s,
     * i.e. at most 7.29e-5 rad after 1 s); hence 1e-4 rad, which is three orders of magnitude below
     * the rotations of up to 0.3 rad being tested.
     */
    private static final double ROTATION_ANGLE_TOLERANCE = 1e-4;

    /**
     * Maximum position error, in meters, right after a Kalman epoch fed with error-free
     * pseudo-ranges of a moving body, as required by the issue: the body moves up to 15 m between
     * epochs, so an epoch using measurements at a timestamp other than their own leaves errors of
     * several meters, while a correct one leaves the solution on the truth.
     */
    private static final double EPOCH_POSITION_TOLERANCE = 0.5;

    /**
     * Gyro noise PSD of the Kalman filter, (0.02 deg/sqrt(hr))^2 expressed in rad^2/s, as used by the
     * tactical grade configuration of Groves' {@code INS_GNSS_Demo_10}.
     */
    private static final double GYRO_NOISE_PSD = Math.pow(Math.toRadians(0.02) / 60.0, 2.0);

    /**
     * Accelerometer noise PSD of the Kalman filter, (200 micro-g/sqrt(Hz))^2 expressed in m^2/s^3,
     * as used by the tactical grade configuration of Groves' {@code INS_GNSS_Demo_10}.
     */
    private static final double ACCELEROMETER_NOISE_PSD = Math.pow(200.0e-6 * 9.80665, 2.0);

    /**
     * Accelerometer bias random walk PSD of the Kalman filter, expressed in m^2/s^5 (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double ACCELEROMETER_BIAS_PSD = 1.0e-7;

    /**
     * Gyro bias random walk PSD of the Kalman filter, expressed in rad^2/s^3 (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double GYRO_BIAS_PSD = 2.0e-12;

    /**
     * Receiver clock frequency-drift PSD of the Kalman filter, expressed in m^2/s^3 (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double CLOCK_FREQUENCY_PSD = 1.0;

    /**
     * Receiver clock phase-drift PSD of the Kalman filter, expressed in m^2/s (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double CLOCK_PHASE_PSD = 1.0;

    /**
     * Pseudo-range measurement noise standard deviation of the Kalman filter, expressed in meters
     * (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double PSEUDO_RANGE_SD = 2.5;

    /**
     * Pseudo-range rate measurement noise standard deviation of the Kalman filter, expressed in
     * meters per second (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double RANGE_RATE_SD = 0.1;

    /**
     * Initial attitude uncertainty per axis, 1 degree expressed in radians (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double INITIAL_ATTITUDE_UNCERTAINTY = Math.toRadians(1.0);

    /**
     * Initial velocity uncertainty per axis, expressed in meters per second (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double INITIAL_VELOCITY_UNCERTAINTY = 0.1;

    /**
     * Initial position uncertainty per axis, expressed in meters (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double INITIAL_POSITION_UNCERTAINTY = 10.0;

    /**
     * Initial accelerometer bias uncertainty per axis, 1000 micro-g expressed in m/s^2 (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double INITIAL_ACCELEROMETER_BIAS_UNCERTAINTY = 1000.0e-6 * 9.80665;

    /**
     * Initial gyro bias uncertainty per axis, 10 deg/hr expressed in rad/s (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double INITIAL_GYRO_BIAS_UNCERTAINTY = Math.toRadians(10.0) / 3600.0;

    /**
     * Initial receiver clock offset uncertainty, expressed in meters (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double INITIAL_CLOCK_OFFSET_UNCERTAINTY = 10.0;

    /**
     * Initial receiver clock drift uncertainty, expressed in meters per second (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double INITIAL_CLOCK_DRIFT_UNCERTAINTY = 0.1;

    /**
     * Number of satellites of the simulated constellation (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final int NUM_SATELLITES = 30;

    /**
     * Orbital radius of the satellites, expressed in meters (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double ORBITAL_RADIUS = 2.656175e7;

    /**
     * Inclination of the satellite orbits, expressed in degrees (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double INCLINATION_DEGREES = 55.0;

    /**
     * Elevation mask angle, expressed in degrees (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double MASK_ANGLE_DEGREES = 10.0;

    /**
     * Signal in space error standard deviation, expressed in meters (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double SIS_ERROR_SD = 1.0;

    /**
     * Zenith ionosphere error standard deviation, expressed in meters (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double ZENITH_IONOSPHERE_ERROR_SD = 2.0;

    /**
     * Zenith troposphere error standard deviation, expressed in meters (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double ZENITH_TROPOSPHERE_ERROR_SD = 0.2;

    /**
     * Code tracking error standard deviation, expressed in meters (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double CODE_TRACKING_ERROR_SD = 1.0;

    /**
     * Range rate tracking error standard deviation, expressed in meters per second (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double RANGE_RATE_TRACKING_ERROR_SD = 0.02;

    /**
     * Receiver clock offset at the start of the noisy simulation, expressed in meters (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double RECEIVER_CLOCK_OFFSET = 10000.0;

    /**
     * Receiver clock drift of the noisy simulation, expressed in meters per second (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double RECEIVER_CLOCK_DRIFT = 100.0;

    /**
     * Seed of the pseudo-random generators, so that every run uses exactly the same inputs.
     */
    private static final long SEED = 36L;

    /**
     * Epoch interval, in seconds, used to check the epoch gate and the timestamp at which
     * measurements are used (GNSS epoch interval of Groves' {@code INS_GNSS_Demo_10}). It is exactly
     * representable in binary floating point, as are the measurement timestamps, so that intervals
     * are exact.
     */
    private static final double GATE_EPOCH_INTERVAL = 0.5;

    /**
     * Timestamp, in seconds, of measurements arriving exactly two epoch intervals after the first
     * ones.
     */
    private static final double GATE_THIRD_TIMESTAMP = 1.0;

    /**
     * Timestamp, in seconds, of measurements arriving only half an epoch interval after the previous
     * ones.
     */
    private static final double GATE_EARLY_TIMESTAMP = 1.25;

    /**
     * Duration, in seconds, of the closed-loop trajectory.
     */
    private static final double CLOSED_LOOP_DURATION = 60.0;

    /**
     * Number of IMU samples between consecutive GNSS measurements (1 s at 100 Hz).
     */
    private static final int SAMPLES_PER_FIX = 100;

    /**
     * Time, in seconds, after which the closed-loop filter is considered converged: ten epochs,
     * enough for the position covariance to settle from the 10 m initial uncertainty.
     */
    private static final double CONVERGENCE_TIME = 10.0;

    /**
     * Number of standard deviations bounding errors in the closed-loop test (99.7 % of the samples
     * of a Gaussian distribution).
     */
    private static final double SIGMA_FACTOR = 3.0;

    /**
     * Minimum ratio of epochs whose position error must lie within three standard deviations of the
     * estimated covariance for the filter to be considered consistent.
     */
    private static final double MIN_CONSISTENT_RATIO = 0.95;

    /**
     * Index of the first position element within the Kalman filter state (attitude, velocity,
     * position, accelerometer bias and gyro bias, three elements each, then clock offset and drift).
     */
    private static final int POSITION_STATE_INDEX = 6;

    /**
     * Number of components of a position.
     */
    private static final int POSITION_COMPONENTS = 3;

    /**
     * Standard gravity, expressed in meters per squared second.
     */
    private static final double STANDARD_GRAVITY = 9.80665;

    /**
     * Conversion from micro-g to meters per squared second.
     */
    private static final double MICRO_G = 1.0e-6 * STANDARD_GRAVITY;

    /**
     * Conversion from degrees per hour to radians per second.
     */
    private static final double DEG_PER_HOUR = Math.toRadians(1.0) / 3600.0;

    /**
     * Accelerometer biases of a tactical grade IMU, expressed in micro-g (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double[] ACCELEROMETER_BIASES_MICRO_G = {900.0, -1300.0, 800.0};

    /**
     * Gyro biases of a tactical grade IMU, expressed in degrees per hour (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double[] GYRO_BIASES_DEG_PER_HOUR = {-9.0, 13.0, -8.0};

    /**
     * Accelerometer scale factor and cross coupling errors of a tactical grade IMU, in row order
     * (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double[] ACCELEROMETER_SCALE_AND_CROSS_COUPLING = {
            500e-6, -300e-6, 200e-6,
            -150e-6, -600e-6, 250e-6,
            -250e-6, 100e-6, 450e-6
    };

    /**
     * Gyro scale factor and cross coupling errors of a tactical grade IMU, in row order (Groves'
     * {@code INS_GNSS_Demo_10}).
     */
    private static final double[] GYRO_SCALE_AND_CROSS_COUPLING = {
            400e-6, -300e-6, 250e-6,
            0.0, -300e-6, -150e-6,
            0.0, 0.0, -350e-6
    };

    /**
     * Gyro g-dependent biases of a tactical grade IMU, in row order and expressed in degrees per hour
     * per g (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double[] GYRO_G_DEPENDENT_BIASES_DEG_PER_HOUR_PER_G = {
            0.9, -1.1, -0.6,
            -0.5, 1.9, -1.6,
            0.3, 1.1, -1.3
    };

    /**
     * Accelerometer noise root PSD of a tactical grade IMU, 100 micro-g/sqrt(Hz) expressed in
     * m * s^-1.5 (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double ACCELEROMETER_NOISE_ROOT_PSD = 100.0 * MICRO_G;

    /**
     * Gyro noise root PSD of a tactical grade IMU, 0.01 deg/sqrt(hr) expressed in rad * s^-0.5
     * (Groves' {@code INS_GNSS_Demo_10}).
     */
    private static final double GYRO_NOISE_ROOT_PSD = Math.toRadians(0.01) / 60.0;

    private int updateGNSSMeasurementsStart;
    private int updateGNSSMeasurementsEnd;
    private int updateBodyKinematicsStart;
    private int updateBodyKinematicsEnd;
    private int propagateStart;
    private int propagateEnd;
    private int reset;

    @Test
    void testConstructor() throws InvalidSourceAndDestinationFrameTypeException {

        // test constructor 1
        var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        var epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        var lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.getLastKinematicsTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastKinematicsTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 2
        final var kalmanConfig = generateKalmanConfig();
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        var kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 3
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                -epochInterval));

        // test constructor 4
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 5
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval));

        // test constructor 6
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());
        
        // test constructor 7
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                -epochInterval, this));

        // test constructor 8
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, this));

        // test constructor 9
        epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());
        
        // test constructor 10
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        final var wrongEpochIntervalTime = new Time(-epochInterval, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime));

        // test constructor 11
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, this));

        // test constructor 12
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, this));

        // test constructor 13
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        var c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 14
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 15
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, 
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                -epochInterval, c));

        // test constructor 16
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 17
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, c));

        // test constructor 18
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 19
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                -epochInterval, c, this));

        // test constructor 20
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, c, this));

        // test constructor 21
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        final var epochIntervalTime2 = new Time(0.0, TimeUnit.MILLISECOND);
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime2,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, c));

        // test constructor 22
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime2,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, c));

        // test constructor 23
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime2,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, c, this));

        // test constructor 24
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime2,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, c, this));

        // test constructor 25
        final var initialConfig = generateInitConfig();
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        INSTightlyCoupledKalmanInitializerConfig initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());
        
        // test constructor 26
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());
        
        // test constructor 27
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig));

        // test constructor 28
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());
        
        // test constructor 29
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig));

        // test constructor 30
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());
        
        // test constructor 31
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, this));

        // test constructor 32
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, this));

        // test constructor 33
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig));

        // test constructor 34
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig));

        // test constructor 35
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, this));

        // test constructor 36
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, this));

        // test constructor 37
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(initialConfig, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 38
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 39
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, c));

        // test constructor 40
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(initialConfig, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 41
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, c));

        // test constructor 42
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 43
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, c, this));

        // test constructor 44
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, c,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, c, this));

        // test constructor 45
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime2, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, c));

        // test constructor 46
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime2, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, c));

        // test constructor 47
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(epochIntervalTime2, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, c, this));

        // test constructor 48
        estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, c,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime2, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSTightlyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, c, this));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetEpochInterval() throws LockedException {
        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator();

        // check default value
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);

        // set a new value
        estimator.setEpochInterval(1.0);

        // check
        assertEquals(1.0, estimator.getEpochInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setEpochInterval(-1.0));
    }

    @Test
    void testGetSetEpochIntervalAsTime() throws LockedException {
        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator();

        final var epochInterval1 = estimator.getEpochIntervalAsTime();

        assertEquals(0.0, epochInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, epochInterval1.getUnit());

        // set a new value
        final var epochInterval2 = new Time(1.0, TimeUnit.SECOND);
        estimator.setEpochInterval(epochInterval2);

        // check
        final var epochInterval3 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochInterval3);
        final var epochInterval4 = estimator.getEpochIntervalAsTime();

        assertEquals(epochInterval2, epochInterval3);
        assertEquals(epochInterval2, epochInterval4);
    }

    @Test
    void testGetSetConfig() throws LockedException {
        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getConfig());
        assertFalse(estimator.getConfig(null));

        // set a new value
        final var config1 = generateKalmanConfig();
        estimator.setConfig(config1);

        // check
        final var config2 = new INSTightlyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(config2));
        final var config3 = estimator.getConfig();

        assertEquals(config1, config2);
        assertEquals(config1, config3);
    }

    @Test
    void testGetSetCoordinateTransformation() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c1 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        estimator.setCoordinateTransformation(c1);

        // check
        final var c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        final var c3 = estimator.getCoordinateTransformation();

        assertEquals(c1, c2);
        assertEquals(c1, c3);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> estimator.setCoordinateTransformation(
                new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)));
    }

    @Test
    void testGetSetInitialConfig() throws LockedException {
        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getInitialConfig());
        assertFalse(estimator.getInitialConfig(null));

        // set a new value
        final var config1 = generateInitConfig();
        estimator.setInitialConfig(config1);

        // check
        final var config2 = new INSTightlyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(config2));
        final var config3 = estimator.getInitialConfig();

        assertEquals(config1, config2);
        assertSame(config1, config3);
    }

    @Test
    void testIsUpdateMeasurementsReady() {
        assertFalse(INSGNSSTightlyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(null));

        final var measurements = new ArrayList<GNSSMeasurement>();
        assertFalse(INSGNSSTightlyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));

        for (var i = 0; i < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS; i++) {
            measurements.add(new GNSSMeasurement());
        }
        assertTrue(INSGNSSTightlyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));
    }

    /**
     * Checks that GNSS measurements provided to an estimator without attitude and without kinematics
     * initialise position, velocity and clock and run the first Kalman epoch: the solution is the true
     * one, the attitude remains the default one, and repeating the timestamp makes no action.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testUpdateMeasurementsWithoutKinematicsAndWithoutInitialAttitude() throws LockedException, NotReadyException,
            INSGNSSException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, false);
        final var measurements = generateMeasurements(timeSeconds, truth);

        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        reset();
        assertListenerCounts(0, 0, 0);

        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));

        // update measurement
        assertTrue(estimator.updateMeasurements(measurements, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
        assertNull(estimator.getLastKinematicsTimestamp());
        assertNull(estimator.getLastKinematicsTimestampAsTime());
        assertFalse(estimator.getLastKinematicsTimestampAsTime(timestamp));
        assertNull(estimator.getKinematics());
        assertNull(estimator.getCorrectedKinematics());

        // attitude is still the default one
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(c.equals(estimator.getCoordinateTransformation(), ABSOLUTE_ERROR));
        final var c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertTrue(c.equals(c2, ABSOLUTE_ERROR));

        final var state1 = assertSolutionEquals(truth, estimator);

        // update again with the same timestamp makes no action
        assertFalse(estimator.updateMeasurements(measurements, new Time(timeSeconds, TimeUnit.SECOND)));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(state1.getGNSSEstimation(), estimator.getEstimation());
        assertEquals(state1, estimator.getState());

        // a Kalman epoch has been run for the measurements, without any IMU sample
        assertListenerCounts(1, 0, 1);
    }

    /**
     * Checks that an IMU sample provided before any GNSS measurement only sets the time reference
     * (there is no position to navigate from), and that measurements provided afterwards initialise the
     * solution to the true one and run the first Kalman epoch.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testUpdateMeasurementsWithBodyKinematicsAndWithoutInitialAttitude() throws LockedException, NotReadyException,
            INSGNSSException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, false);
        final var measurements = generateMeasurements(2.0 * timeSeconds, truth);

        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertKinematicsBeforeMeasurements(estimator, c, timeSeconds);

        reset();
        assertListenerCounts(0, 0, 0);

        // update measurement
        assertTrue(estimator.updateMeasurements(measurements, 2.0 * timeSeconds));

        assertMeasurementsAfterKinematics(estimator, measurements, c, timeSeconds);
        assertSolutionEquals(truth, estimator);

        // a Kalman epoch has been run for the measurements
        assertListenerCounts(1, 0, 1);
    }

    /**
     * Checks that GNSS measurements provided to an estimator with an initial attitude and without
     * kinematics initialise position, velocity and clock and run the first Kalman epoch, keeping the
     * initial attitude.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testUpdateMeasurementsWithoutBodyKinematicsAndWithInitialAttitude() throws LockedException, NotReadyException,
            INSGNSSException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, true);
        final var c = truth.getCoordinateTransformation();
        final var measurements = generateMeasurements(timeSeconds, truth);

        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), c, this);

        reset();
        assertListenerCounts(0, 0, 0);

        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertEquals(c, estimator.getCoordinateTransformation());
        var c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);

        // update measurement
        assertTrue(estimator.updateMeasurements(measurements, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
        assertNull(estimator.getLastKinematicsTimestamp());
        assertNull(estimator.getKinematics());
        assertNull(estimator.getCorrectedKinematics());

        // initial attitude is kept
        assertTrue(c.equals(estimator.getCoordinateTransformation(), ABSOLUTE_ERROR));
        c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertTrue(c.equals(c2, ABSOLUTE_ERROR));

        final var state1 = assertSolutionEquals(truth, estimator);
        assertTrue(truth.getCoordinateTransformationMatrix().equals(
                state1.getBodyToEcefCoordinateTransformationMatrix(), ABSOLUTE_ERROR));

        assertListenerCounts(1, 0, 1);
    }

    /**
     * Checks that an IMU sample provided before any GNSS measurement to an estimator with an initial
     * attitude only sets the time reference, and that measurements provided afterwards initialise the
     * solution to the true one and run the first Kalman epoch, keeping the initial attitude.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testUpdateMeasurementsWithKinematicsAndInitialAttitude() throws LockedException, NotReadyException,
            INSGNSSException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, true);
        final var c = truth.getCoordinateTransformation();
        final var measurements = generateMeasurements(2.0 * timeSeconds, truth);

        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), c, this);

        assertKinematicsBeforeMeasurements(estimator, c, timeSeconds);

        reset();
        assertListenerCounts(0, 0, 0);

        // update measurement
        assertTrue(estimator.updateMeasurements(measurements, 2.0 * timeSeconds));

        assertMeasurementsAfterKinematics(estimator, measurements, c, timeSeconds);
        final var state1 = assertSolutionEquals(truth, estimator);
        assertTrue(truth.getCoordinateTransformationMatrix().equals(
                state1.getBodyToEcefCoordinateTransformationMatrix(), ABSOLUTE_ERROR));

        assertListenerCounts(1, 0, 1);
    }

    @Test
    void testUpdateMeasurementsWhenNotReadyThrowsNotReadyException() {
        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();
        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig);

        assertThrows(NotReadyException.class, () -> estimator.updateMeasurements(Collections.emptyList(),
                0.0));
    }

    /**
     * Checks that with zero specific force and zero angular rate the body falls freely from the
     * solution initialised with GNSS measurements, keeping its attitude.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testUpdateKinematicsWithZeroSpecificForceAndAngularRate() throws LockedException, NotReadyException,
            INSGNSSException, InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, false);

        final var integration = assertUpdateBodyKinematicsIntegratesSample(truth, new BodyKinematics(),
                timeSeconds);

        assertFreeFall(integration.initialFrame(), integration.result(), timeSeconds);
        assertEquals(0.0, rotationAngle(integration.initialFrame(), integration.result()),
                ROTATION_ANGLE_TOLERANCE);
    }

    /**
     * Checks that a specific force equal to gravity (expressed in body axes, which without attitude
     * are the ECEF axes) and zero angular rate are integrated with the ECEF inertial navigation
     * equations, keeping the attitude.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testUpdateKinematicsWithFreeFallSpecificForceAndZeroAngularRate() throws LockedException, NotReadyException,
            INSGNSSException, InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, false);

        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(truth.getX(), truth.getY(),
                truth.getZ());

        // because there is no attitude, the specific force is directly the gravity.
        final var kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                0.0, 0.0, 0.0);
        final var integration = assertUpdateBodyKinematicsIntegratesSample(truth, kinematics, timeSeconds);

        assertEquals(0.0, rotationAngle(integration.initialFrame(), integration.result()),
                ROTATION_ANGLE_TOLERANCE);
    }

    /**
     * Checks that with zero specific force and a non-zero angular rate the body falls freely while its
     * attitude rotates by the angular rate times the elapsed time.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testUpdateKinematicsWithZeroSpecificForceAndRotationOnly() throws LockedException, NotReadyException,
            INSGNSSException, InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, false);

        final var kinematics = createRotatingKinematics(randomizer, 0.0, 0.0, 0.0);
        final var integration = assertUpdateBodyKinematicsIntegratesSample(truth, kinematics, timeSeconds);

        assertFreeFall(integration.initialFrame(), integration.result(), timeSeconds);
        assertEquals(kinematics.getAngularRateNorm() * timeSeconds,
                rotationAngle(integration.initialFrame(), integration.result()), ROTATION_ANGLE_TOLERANCE);
    }

    /**
     * Checks that a specific force equal to gravity and a non-zero angular rate are integrated with the
     * ECEF inertial navigation equations, and that the attitude rotates by the angular rate times the
     * elapsed time.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testUpdateKinematicsWithFreeFallSpecificForceAndRotation() throws LockedException, NotReadyException,
            INSGNSSException, InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, false);

        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(truth.getX(), truth.getY(),
                truth.getZ());

        // because there is no attitude, the specific force is directly the gravity.
        final var kinematics = createRotatingKinematics(randomizer, gravity.getGx(), gravity.getGy(),
                gravity.getGz());
        final var integration = assertUpdateBodyKinematicsIntegratesSample(truth, kinematics, timeSeconds);

        assertEquals(kinematics.getAngularRateNorm() * timeSeconds,
                rotationAngle(integration.initialFrame(), integration.result()), ROTATION_ANGLE_TOLERANCE);
    }

    /**
     * Checks that propagate never uses again GNSS measurements already consumed by a Kalman epoch (it
     * returns false and leaves the state untouched), and that it runs the pending epoch when
     * measurements were provided before the estimator was ready to propagate.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testPropagate() throws LockedException, NotReadyException,
            INSGNSSException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, false);
        final var measurements = generateMeasurements(timeSeconds, truth);

        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        reset();
        assertListenerCounts(0, 0, 0);

        // update measurement
        assertTrue(estimator.updateMeasurements(measurements, timeSeconds));

        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        final var state1 = assertSolutionEquals(truth, estimator);
        assertListenerCounts(1, 0, 1);

        // measurements have already been consumed by an epoch: propagate makes no action
        assertFalse(estimator.propagate(2.0 * timeSeconds));
        assertFalse(estimator.propagate(new Time(2.0 * timeSeconds, TimeUnit.SECOND)));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(state1.getGNSSEstimation(), estimator.getEstimation());
        assertEquals(state1, estimator.getState());
        assertListenerCounts(1, 0, 1);

        // when measurements arrive before a Kalman configuration is available, only the least squares
        // solution is computed and measurements remain pending
        final var estimator2 = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createInitConfig(), this);

        reset();
        assertFalse(estimator2.isPropagateReady());
        assertTrue(estimator2.updateMeasurements(measurements, timeSeconds));

        assertNotNull(estimator2.getEstimation());
        assertNull(estimator2.getState());
        assertNull(estimator2.getLastStateTimestamp());
        assertFalse(estimator2.isPropagateReady());
        assertFalse(estimator2.propagate(timeSeconds));
        assertListenerCounts(1, 0, 0);

        // once ready, propagate runs the pending epoch
        estimator2.setConfig(createKalmanConfig());
        assertTrue(estimator2.isPropagateReady());
        assertTrue(estimator2.propagate(new Time(timeSeconds, TimeUnit.SECOND)));

        assertFalse(estimator2.isRunning());
        assertEquals(timeSeconds, estimator2.getLastStateTimestamp(), 0.0);
        final var state2 = assertSolutionEquals(truth, estimator2);
        assertListenerCounts(1, 0, 1);

        // and the same measurements are not used again
        assertFalse(estimator2.propagate(2.0 * timeSeconds));

        assertEquals(timeSeconds, estimator2.getLastStateTimestamp(), 0.0);
        assertEquals(state2, estimator2.getState());
        assertListenerCounts(1, 0, 1);
    }

    @Test
    void testPropagateWhenNotReadyReturnsFalse() throws LockedException, INSGNSSException {
        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();

        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig);

        assertFalse(estimator.propagate(0.0));
    }

    /**
     * Checks that reset discards measurements, kinematics, solution, Kalman filter state and time
     * references, so that the same measurements are accepted again and give the same result.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testReset() throws LockedException, NotReadyException,
            INSGNSSException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var truth = createFrame(randomizer, false);
        final var measurements = generateMeasurements(timeSeconds, truth);

        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        reset();
        assertListenerCounts(0, 0, 0);

        // update measurement and kinematics
        assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
        assertTrue(estimator.updateBodyKinematics(new BodyKinematics(), timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertNotNull(estimator.getKinematics());
        assertNotNull(estimator.getCorrectedKinematics());
        assertNotNull(estimator.getCoordinateTransformation());
        final var state1 = assertSolutionEquals(truth, estimator);

        // reset
        assertEquals(0, reset);

        estimator.reset();

        assertEquals(1, reset);
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertNull(estimator.getState());
        assertNull(estimator.getLastStateTimestamp());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertNull(estimator.getCoordinateTransformation());
        assertNull(estimator.getKinematics());
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.propagate(timeSeconds));

        // update again with the same timestamp now it does make an action
        assertTrue(estimator.updateMeasurements(measurements, new Time(timeSeconds, TimeUnit.SECOND)));

        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(state1.getGNSSEstimation(), estimator.getEstimation());
        assertEquals(state1, estimator.getState());

        assertListenerCounts(2, 1, 2);
    }

    /**
     * Checks that the angular rate of every IMU sample is integrated: after a 10 s trajectory yawing at
     * 0.1 rad/s, with GNSS measurements provided only at the start, the estimated yaw must be 1 rad.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testRotationIsIntegrated() throws LockedException, NotReadyException, INSGNSSException,
            InvalidSourceAndDestinationFrameTypeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.yawingStationary(YAW_RATE, TRAJECTORY_DURATION);
        final var estimator = createEstimator(0.0, trajectory);
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);

        initialize(estimator, trajectory);
        feed(estimator, trajectory);

        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(currentFrame(estimator));
        assertEquals(EXPECTED_FINAL_YAW, nedFrame.getCoordinateTransformation().getYawEulerAngle(), YAW_TOLERANCE);
    }

    /**
     * Checks that IMU samples provided before the first GNSS measurements are integrated: rotation
     * occurred before position and velocity are initialised must not be lost.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testRotationBeforeFirstMeasurementsIsIntegrated() throws LockedException, NotReadyException,
            INSGNSSException, InvalidSourceAndDestinationFrameTypeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.yawingStationary(YAW_RATE, TRAJECTORY_DURATION);
        final var estimator = createEstimator(0.0, trajectory);

        // no GNSS measurements are provided until the end of the trajectory
        feed(estimator, trajectory);
        assertNull(estimator.getEstimation());

        final var timestamp = trajectory.getTimestamp(trajectory.getKinematics().size());
        assertTrue(estimator.updateMeasurements(generateMeasurements(timestamp, trajectory.getFinalFrame()),
                timestamp));

        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(currentFrame(estimator));
        assertEquals(EXPECTED_FINAL_YAW, nedFrame.getCoordinateTransformation().getYawEulerAngle(), YAW_TOLERANCE);
    }

    /**
     * Checks that the specific force of every IMU sample is integrated: after moving north at 10 m/s
     * for 10 s, with GNSS measurements provided only at the start, the estimated position and velocity
     * must be the true ones (up to the accuracy of the GNSS initialisation), and must equal the
     * per-sample integration from the initialised solution.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testTranslationIsIntegrated() throws LockedException, NotReadyException, INSGNSSException,
            InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.northAtConstantSpeed(NORTH_SPEED,
                TRAJECTORY_DURATION);
        final var estimator = createEstimator(0.0, trajectory);

        initialize(estimator, trajectory);
        final var initialFrame = currentFrame(estimator);
        feed(estimator, trajectory);

        // the solution is the true one up to the accuracy of the GNSS initialisation
        final var truth = trajectory.getFinalFrame();
        final var result = currentFrame(estimator);
        assertEquals(0.0, positionError(truth, result), GNSS_POSITION_TOLERANCE);
        assertEquals(0.0, velocityError(truth, result), GNSS_VELOCITY_TOLERANCE);

        // and it is exactly the integration of every sample from the initialised solution
        final var expected = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame,
                trajectory.getKinematics(), trajectory.getSampleInterval());
        assertFrameEquals(expected, result);
        assertEquals(NORTH_SPEED * TRAJECTORY_DURATION, positionError(initialFrame, result),
                GNSS_POSITION_TOLERANCE);
    }

    /**
     * Checks that IMU samples arriving within a Kalman epoch interval are all integrated: with a 1 s
     * epoch interval and 100 Hz samples of a body that accelerates and then decelerates, the solution
     * must equal the per-sample integration, not a single step using only the last sample.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testEveryImuSampleCountsWithinEpoch() throws LockedException, NotReadyException, INSGNSSException,
            InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.accelerateThenDecelerate(HALF_EPOCH_DURATION,
                HALF_EPOCH_DURATION);
        final var estimator = createEstimator(ONE_SECOND_EPOCH_INTERVAL, trajectory);

        initialize(estimator, trajectory);
        final var initialFrame = currentFrame(estimator);
        feed(estimator, trajectory);

        final var kinematics = trajectory.getKinematics();
        final var expected = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame, kinematics,
                trajectory.getSampleInterval());
        final var result = currentFrame(estimator);
        assertFrameEquals(expected, result);

        // a single step over the whole epoch using only the last sample gives a different solution
        final var singleStep = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(ONE_SECOND_EPOCH_INTERVAL, initialFrame, kinematics.getLast(),
                singleStep);
        assertTrue(positionError(singleStep, result) > DIFFERENT_POSITION_THRESHOLD);
    }

    /**
     * Checks that GNSS measurements are used in a Kalman epoch at their own timestamp: for a body
     * whose speed and heading change, error-free 100 Hz kinematics and error-free pseudo-ranges every
     * second, no Kalman epoch is run between measurements and the position error right after each
     * epoch is below 0.5 m.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testMeasurementsAreUsedAtTheirTimestamp() throws LockedException, NotReadyException, INSGNSSException,
            InvalidSourceAndDestinationFrameTypeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.turningAndAccelerating(TRAJECTORY_DURATION);
        final var frames = trajectory.getFrames();
        final var kinematics = trajectory.getKinematics();
        final var gnssConfig = createGnssConfig(false);
        final var random = new Random(SEED);
        final var biases = generateBiases(trajectory.getInitialFrame(), gnssConfig, random);

        final var estimator = createEstimator(GATE_EPOCH_INTERVAL, trajectory);
        initialize(estimator, trajectory);

        var lastEpochTimestamp = trajectory.getTimestamp(0);
        var numEpochs = 0;
        estimator.updateBodyKinematics(kinematics.getFirst(), trajectory.getTimestamp(0));
        for (var i = 1; i < frames.size(); i++) {
            final var timestamp = trajectory.getTimestamp(i);
            estimator.updateBodyKinematics(kinematics.get(i - 1), timestamp);

            // IMU samples do not run Kalman epochs: the last epoch is the one of the last measurements
            assertEquals(lastEpochTimestamp, estimator.getLastStateTimestamp(), 0.0);

            if (i % SAMPLES_PER_FIX == 0) {
                final var truth = frames.get(i);
                assertTrue(estimator.updateMeasurements(
                        generateMeasurements(timestamp, truth, biases, gnssConfig, random), timestamp));

                assertEquals(timestamp, estimator.getLastStateTimestamp(), 0.0);
                assertTrue(positionError(truth, currentFrame(estimator)) < EPOCH_POSITION_TOLERANCE);

                lastEpochTimestamp = timestamp;
                numEpochs++;
            }
        }

        assertEquals(Math.round(TRAJECTORY_DURATION), numEpochs);
    }

    /**
     * Checks the closed loop on a trajectory with speed and heading changes, tactical grade IMU errors
     * and noisy pseudo-ranges every second: after convergence the position error stays below three
     * standard deviations of the GNSS position accuracy, it ends up smaller than the error of the
     * INS-only solution for the same data, and it is consistent with the estimated covariance.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InertialNavigatorException                    if the INS-only navigation fails.
     * @throws WrongSizeException                            never thrown.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testClosedLoopTracksTruth() throws LockedException, NotReadyException, INSGNSSException,
            InertialNavigatorException, WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.turningAndAccelerating(CLOSED_LOOP_DURATION);
        final var frames = trajectory.getFrames();
        final var gnssConfig = createGnssConfig(true);

        final var random = new Random(SEED);
        final var measuredKinematics = new ArrayList<BodyKinematics>();
        BodyKinematicsGenerator.generate(trajectory.getSampleInterval(), trajectory.getKinematics(),
                createImuErrors(), random, measuredKinematics);
        final var biases = generateBiases(trajectory.getInitialFrame(), gnssConfig, random);

        final var estimator = createEstimator(0.0, trajectory);
        assertTrue(estimator.updateMeasurements(generateMeasurements(trajectory.getTimestamp(0),
                trajectory.getInitialFrame(), biases, gnssConfig, random), trajectory.getTimestamp(0)));
        final var initialFrame = currentFrame(estimator);

        var numEpochs = 0;
        var numConsistent = 0;
        var maxConvergedError = 0.0;
        estimator.updateBodyKinematics(measuredKinematics.getFirst(), trajectory.getTimestamp(0));
        for (var i = 1; i < frames.size(); i++) {
            final var timestamp = trajectory.getTimestamp(i);
            estimator.updateBodyKinematics(measuredKinematics.get(i - 1), timestamp);

            if (i % SAMPLES_PER_FIX == 0) {
                // noisy pseudo-ranges every second
                final var truth = frames.get(i);
                assertTrue(estimator.updateMeasurements(
                        generateMeasurements(timestamp, truth, biases, gnssConfig, random), timestamp));

                final var state = estimator.getState();
                numEpochs++;
                if (isPositionWithinCovariance(truth, state)) {
                    numConsistent++;
                }
                if (timestamp >= CONVERGENCE_TIME) {
                    maxConvergedError = Math.max(maxConvergedError, positionError(truth, currentFrame(estimator)));
                }
            }
        }

        // after convergence the position error stays below three standard deviations of the fix
        assertEquals(Math.round(CLOSED_LOOP_DURATION), numEpochs);
        assertTrue(maxConvergedError < SIGMA_FACTOR * PSEUDO_RANGE_SD);

        // the filtered solution ends up closer to the truth than the INS-only one
        final var truth = trajectory.getFinalFrame();
        final var insOnly = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame, measuredKinematics,
                trajectory.getSampleInterval());
        assertTrue(positionError(truth, currentFrame(estimator)) < positionError(truth, insOnly));

        // position errors are consistent with the estimated covariance
        assertTrue(numConsistent >= MIN_CONSISTENT_RATIO * numEpochs);
    }

    /**
     * Checks the epoch gate of GNSS measurements: measurements arriving exactly one epoch interval
     * apart are accepted, while measurements arriving earlier, or repeating a timestamp, are ignored.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testEpochGateAcceptsExactInterval() throws LockedException, NotReadyException, INSGNSSException,
            InvalidSourceAndDestinationFrameTypeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, GATE_THIRD_TIMESTAMP);
        final var truth = trajectory.getInitialFrame();
        final var gnssConfig = createGnssConfig(false);
        final var random = new Random(SEED);
        final var biases = generateBiases(truth, gnssConfig, random);

        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                GATE_EPOCH_INTERVAL, createInitConfig(), truth.getCoordinateTransformation(), this);
        reset();

        assertTrue(estimator.updateMeasurements(generateMeasurements(0.0, truth, biases, gnssConfig, random), 0.0));
        assertEquals(0.0, estimator.getLastStateTimestamp(), 0.0);

        assertTrue(estimator.updateMeasurements(
                generateMeasurements(GATE_EPOCH_INTERVAL, truth, biases, gnssConfig, random), GATE_EPOCH_INTERVAL));
        assertEquals(GATE_EPOCH_INTERVAL, estimator.getLastStateTimestamp(), 0.0);

        assertTrue(estimator.updateMeasurements(
                generateMeasurements(GATE_THIRD_TIMESTAMP, truth, biases, gnssConfig, random),
                new Time(GATE_THIRD_TIMESTAMP, TimeUnit.SECOND)));
        assertEquals(GATE_THIRD_TIMESTAMP, estimator.getLastStateTimestamp(), 0.0);

        final var state = estimator.getState();
        final var measurements = generateMeasurements(GATE_EARLY_TIMESTAMP, truth, biases, gnssConfig, random);

        // measurements arriving before a whole epoch interval has elapsed are ignored
        assertFalse(estimator.updateMeasurements(measurements, GATE_EARLY_TIMESTAMP));
        // measurements repeating the timestamp of the last epoch are ignored
        assertFalse(estimator.updateMeasurements(measurements, GATE_THIRD_TIMESTAMP));
        // measurements older than the last epoch are ignored
        assertFalse(estimator.updateMeasurements(measurements, GATE_EPOCH_INTERVAL));

        assertEquals(GATE_THIRD_TIMESTAMP, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(state, estimator.getState());
        assertEquals(3, updateGNSSMeasurementsStart);
        assertEquals(3, updateGNSSMeasurementsEnd);
        assertEquals(3, propagateStart);
        assertEquals(3, propagateEnd);
    }

    @Override
    public void onUpdateGNSSMeasurementsStart(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateGNSSMeasurementsStart++;
    }

    @Override
    public void onUpdateGNSSMeasurementsEnd(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateGNSSMeasurementsEnd++;
    }

    @Override
    public void onUpdateBodyKinematicsStart(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateBodyKinematicsStart++;
    }

    @Override
    public void onUpdateBodyKinematicsEnd(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateBodyKinematicsEnd++;
    }

    @Override
    public void onPropagateStart(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateStart++;
    }

    @Override
    public void onPropagateEnd(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateEnd++;
    }

    @Override
    public void onReset(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        reset++;
    }

    private void reset() {
        updateGNSSMeasurementsStart = 0;
        updateGNSSMeasurementsEnd = 0;
        updateBodyKinematicsStart = 0;
        updateBodyKinematicsEnd = 0;
        propagateStart = 0;
        propagateEnd = 0;
        reset = 0;
    }

    private static void checkLocked(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setConfig(null));
        assertThrows(LockedException.class, () -> estimator.setCoordinateTransformation(null));
        assertThrows(LockedException.class, () -> estimator.setInitialConfig(null));
        assertThrows(LockedException.class, () -> estimator.updateMeasurements(null, 0.0));
        assertThrows(LockedException.class, () -> estimator.updateMeasurements(null,
                new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.updateBodyKinematics(null, 0.0));
        assertThrows(LockedException.class, () -> estimator.updateBodyKinematics(null,
                new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.propagate(0.0));
        assertThrows(LockedException.class, () -> estimator.propagate(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, estimator::reset);
    }

    private static INSTightlyCoupledKalmanInitializerConfig generateInitConfig() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty);
    }

    private static INSTightlyCoupledKalmanConfig generateKalmanConfig() {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
    }

    /**
     * Creates an estimator with the deterministic tactical grade configuration and the true attitude
     * at the start of a trajectory. Position, velocity and clock are left to be initialised from GNSS
     * measurements.
     *
     * @param epochInterval minimum epoch interval expressed in seconds.
     * @param trajectory    trajectory providing the initial attitude.
     * @return a new estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    private static INSGNSSTightlyCoupledKalmanFilteredEstimator createEstimator(
            final double epochInterval, final KalmanFilteredEstimatorTrajectory trajectory)
            throws InvalidSourceAndDestinationFrameTypeException {
        return new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(), epochInterval,
                createInitConfig(), trajectory.getInitialFrame().getCoordinateTransformation());
    }

    /**
     * Initialises the position, velocity and clock of an estimator by providing error-free GNSS
     * measurements for the true frame at the start of a trajectory.
     *
     * @param estimator  estimator to be initialised.
     * @param trajectory trajectory providing the true initial frame.
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSGNSSException  if estimation fails.
     */
    private static void initialize(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator,
            final KalmanFilteredEstimatorTrajectory trajectory) throws LockedException, NotReadyException,
            INSGNSSException {
        final var timestamp = trajectory.getTimestamp(0);
        assertTrue(estimator.updateMeasurements(generateMeasurements(timestamp, trajectory.getInitialFrame()),
                timestamp));
    }

    /**
     * Feeds the kinematics of a whole trajectory to an estimator: the first kinematics sample is
     * provided at the start time (which only sets the time reference), then one sample at the end of
     * each sample interval.
     *
     * @param estimator  estimator to be fed.
     * @param trajectory trajectory providing the error-free kinematics.
     * @return number of samples accepted by the estimator.
     * @throws LockedException  if estimator is locked.
     * @throws INSGNSSException if estimation fails.
     */
    private static int feed(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator,
            final KalmanFilteredEstimatorTrajectory trajectory) throws LockedException, INSGNSSException {
        final var kinematics = trajectory.getKinematics();
        var accepted = estimator.updateBodyKinematics(kinematics.getFirst(), trajectory.getTimestamp(0)) ? 1 : 0;
        for (var i = 0; i < kinematics.size(); i++) {
            if (estimator.updateBodyKinematics(kinematics.get(i), trajectory.getTimestamp(i + 1))) {
                accepted++;
            }
        }
        return accepted;
    }

    /**
     * Gets the current solution of an estimator as a frame, built from its current estimation
     * (position and velocity) and its current body to ECEF coordinate transformation (attitude).
     *
     * @param estimator estimator whose solution is requested.
     * @return current solution of the estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    private static ECEFFrame currentFrame(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator)
            throws InvalidSourceAndDestinationFrameTypeException {
        final var estimation = estimator.getEstimation();
        return new ECEFFrame(estimation.getX(), estimation.getY(), estimation.getZ(),
                estimation.getVx(), estimation.getVy(), estimation.getVz(), estimator.getCoordinateTransformation());
    }

    /**
     * Creates the configuration of the simulated GNSS constellation and receiver (values of Groves'
     * {@code INS_GNSS_Demo_10}).
     *
     * @param noisy true to simulate signal in space, atmosphere and tracking errors and a receiver
     *              clock offset and drift, false to generate error-free measurements of a receiver
     *              with a perfect clock.
     * @return GNSS configuration.
     */
    private static GNSSConfig createGnssConfig(final boolean noisy) {
        if (noisy) {
            return new GNSSConfig(GATE_EPOCH_INTERVAL, 0.0, 0.0, 0.0, NUM_SATELLITES, ORBITAL_RADIUS,
                    INCLINATION_DEGREES, 0.0, 0.0, MASK_ANGLE_DEGREES, 0.0, 0.0, 0.0, PSEUDO_RANGE_SD, RANGE_RATE_SD,
                    RECEIVER_CLOCK_OFFSET, RECEIVER_CLOCK_DRIFT);
        } else {
            return new GNSSConfig(GATE_EPOCH_INTERVAL, 0.0, 0.0, 0.0, NUM_SATELLITES, ORBITAL_RADIUS,
                    INCLINATION_DEGREES, 0.0, 0.0, MASK_ANGLE_DEGREES, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
    }

    /**
     * Generates the range biases of every satellite of the constellation, which as in Groves'
     * simulation are kept constant for a whole run.
     *
     * @param truth      true frame at the start of the run.
     * @param gnssConfig GNSS configuration.
     * @param random     seeded random number generator.
     * @return range bias of each satellite, expressed in meters.
     */
    private static List<Double> generateBiases(
            final ECEFFrame truth, final GNSSConfig gnssConfig, final Random random) {
        final var satellitePositions = new ArrayList<ECEFPosition>();
        for (final var satellite : SatelliteECEFPositionAndVelocityGenerator.generateSatellitesPositionAndVelocity(
                0.0, gnssConfig)) {
            satellitePositions.add(satellite.getEcefPosition());
        }
        return GNSSBiasesGenerator.generateBiases(satellitePositions, truth.getECEFPosition(), gnssConfig, random);
    }

    /**
     * Generates the GNSS measurements (pseudo-ranges and pseudo-range rates) of the satellites in
     * view from a true frame at a given time.
     *
     * @param timestamp  time expressed in seconds.
     * @param truth      true frame at that time.
     * @param biases     range bias of each satellite.
     * @param gnssConfig GNSS configuration.
     * @param random     seeded random number generator.
     * @return GNSS measurements of the satellites above the mask angle.
     */
    private static Collection<GNSSMeasurement> generateMeasurements(
            final double timestamp, final ECEFFrame truth, final List<Double> biases, final GNSSConfig gnssConfig,
            final Random random) {
        final var satellites = new ArrayList<>(
                SatelliteECEFPositionAndVelocityGenerator.generateSatellitesPositionAndVelocity(timestamp,
                        gnssConfig));
        final var measurements = GNSSMeasurementsGenerator.generate(timestamp, satellites, truth.getECEFPosition(),
                truth.getECEFVelocity(), biases, gnssConfig, random);
        assertTrue(measurements.size() >= GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS);
        return measurements;
    }

    /**
     * Asserts that two frames are equal up to the tolerances of the chained navigator oracle.
     *
     * @param expected expected frame.
     * @param actual   actual frame.
     */
    private static void assertFrameEquals(final ECEFFrame expected, final ECEFFrame actual) {
        assertEquals(0.0, positionError(expected, actual), CHAIN_POSITION_TOLERANCE);
        assertEquals(0.0, velocityError(expected, actual), CHAIN_VELOCITY_TOLERANCE);
        assertTrue(expected.getCoordinateTransformationMatrix().equals(actual.getCoordinateTransformationMatrix(),
                CHAIN_ROTATION_TOLERANCE));
    }

    /**
     * Distance between the positions of two frames.
     *
     * @param expected expected frame.
     * @param actual   actual frame.
     * @return distance expressed in meters.
     */
    private static double positionError(final ECEFFrame expected, final ECEFFrame actual) {
        return norm(actual.getX() - expected.getX(), actual.getY() - expected.getY(),
                actual.getZ() - expected.getZ());
    }

    /**
     * Norm of the difference between the velocities of two frames.
     *
     * @param expected expected frame.
     * @param actual   actual frame.
     * @return velocity difference expressed in meters per second.
     */
    private static double velocityError(final ECEFFrame expected, final ECEFFrame actual) {
        return norm(actual.getVx() - expected.getVx(), actual.getVy() - expected.getVy(),
                actual.getVz() - expected.getVz());
    }

    /**
     * Euclidean norm of a 3D vector.
     *
     * @param x x coordinate.
     * @param y y coordinate.
     * @param z z coordinate.
     * @return norm of the vector.
     */
    private static double norm(final double x, final double y, final double z) {
        return Math.sqrt(x * x + y * y + z * z);
    }

    /**
     * Generates the error-free GNSS measurements of the satellites in view from a true frame at a given
     * time, for a receiver with a perfect clock.
     *
     * @param timestamp time expressed in seconds.
     * @param truth     true frame at that time.
     * @return GNSS measurements of the satellites above the mask angle.
     */
    private static Collection<GNSSMeasurement> generateMeasurements(final double timestamp, final ECEFFrame truth) {
        final var gnssConfig = createGnssConfig(false);
        final var random = new Random(SEED);
        return generateMeasurements(timestamp, truth, generateBiases(truth, gnssConfig, random), gnssConfig,
                random);
    }

    /**
     * Creates a true frame with a seeded pseudo-random position and velocity.
     *
     * @param randomizer     seeded randomizer.
     * @param randomAttitude true to also use a seeded pseudo-random attitude, false to use the default
     *                       attitude (body axes aligned with ECEF axes).
     * @return a new ECEF frame.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    private static ECEFFrame createFrame(final UniformRandomizer randomizer, final boolean randomAttitude)
            throws InvalidSourceAndDestinationFrameTypeException {
        final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
        final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

        final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

        final var ecefUserPosition = new ECEFPosition();
        final var ecefUserVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                ecefUserVelocity);

        final CoordinateTransformation c;
        if (randomAttitude) {
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        } else {
            c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        }

        return new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);
    }

    /**
     * Creates body kinematics with a given specific force and a seeded pseudo-random angular rate.
     *
     * @param randomizer seeded randomizer.
     * @param fx         x coordinate of specific force expressed in meters per squared second.
     * @param fy         y coordinate of specific force expressed in meters per squared second.
     * @param fz         z coordinate of specific force expressed in meters per squared second.
     * @return body kinematics.
     */
    private static BodyKinematics createRotatingKinematics(
            final UniformRandomizer randomizer, final double fx, final double fy, final double fz) {
        final var angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                MAX_DEGREES_PER_SECOND));
        final var angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                MAX_DEGREES_PER_SECOND));
        final var angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                MAX_DEGREES_PER_SECOND));
        return new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Asserts the number of times each pair of listener events has been notified.
     *
     * @param expectedUpdateGNSSMeasurements number of GNSS measurement updates.
     * @param expectedUpdateBodyKinematics   number of body kinematics updates.
     * @param expectedPropagate              number of Kalman epochs.
     */
    private void assertListenerCounts(
            final int expectedUpdateGNSSMeasurements, final int expectedUpdateBodyKinematics,
            final int expectedPropagate) {
        assertEquals(expectedUpdateGNSSMeasurements, updateGNSSMeasurementsStart);
        assertEquals(expectedUpdateGNSSMeasurements, updateGNSSMeasurementsEnd);
        assertEquals(expectedUpdateBodyKinematics, updateBodyKinematicsStart);
        assertEquals(expectedUpdateBodyKinematics, updateBodyKinematicsEnd);
        assertEquals(expectedPropagate, propagateStart);
        assertEquals(expectedPropagate, propagateEnd);
    }

    /**
     * Asserts that the estimation and the Kalman filter state of an estimator are consistent with each
     * other and hold the true position and velocity up to the accuracy of an initialisation from
     * error-free GNSS measurements.
     *
     * @param truth     true frame.
     * @param estimator estimator to be checked.
     * @return current Kalman filter state of the estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    private static INSTightlyCoupledKalmanState assertSolutionEquals(
            final ECEFFrame truth, final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator)
            throws InvalidSourceAndDestinationFrameTypeException {
        final var estimation1 = estimator.getEstimation();
        assertNotNull(estimation1);

        final var estimation2 = new GNSSEstimation();
        assertTrue(estimator.getEstimation(estimation2));
        assertEquals(estimation1, estimation2);

        final var state1 = estimator.getState();
        assertNotNull(state1);

        final var state2 = new INSTightlyCoupledKalmanState();
        assertTrue(estimator.getState(state2));
        assertEquals(state1, state2);

        assertEquals(estimation1, state1.getGNSSEstimation());

        final var result = currentFrame(estimator);
        assertEquals(0.0, positionError(truth, result), GNSS_POSITION_TOLERANCE);
        assertEquals(0.0, velocityError(truth, result), GNSS_VELOCITY_TOLERANCE);

        return state1;
    }

    /**
     * Checks an IMU sample provided before any GNSS measurement: it is accepted and stored, but there
     * is no solution to navigate from, so only the time reference is set and no Kalman filter state is
     * created.
     *
     * @param estimator   estimator to be checked, to which no measurement has been provided.
     * @param c           attitude of the estimator.
     * @param timeSeconds timestamp of the sample expressed in seconds.
     * @throws LockedException  if estimator is locked.
     * @throws INSGNSSException if estimation fails.
     */
    private static void assertKinematicsBeforeMeasurements(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator, final CoordinateTransformation c,
            final double timeSeconds) throws LockedException, INSGNSSException {
        final var kinematics = new BodyKinematics();
        assertTrue(estimator.updateBodyKinematics(kinematics, timeSeconds));

        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastKinematicsTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastKinematicsTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
        assertEquals(kinematics, estimator.getKinematics());
        final var kinematics2 = new BodyKinematics();
        assertTrue(estimator.getKinematics(kinematics2));
        assertEquals(kinematics, kinematics2);
        assertEquals(kinematics, estimator.getCorrectedKinematics());
        assertTrue(estimator.getCorrectedKinematics(kinematics2));
        assertEquals(kinematics, kinematics2);
        assertEquals(c, estimator.getCoordinateTransformation());
        final var c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);

        // a sample that does not advance time is ignored
        assertFalse(estimator.updateBodyKinematics(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
    }

    /**
     * Checks the state of an estimator right after providing GNSS measurements at twice the timestamp
     * of a previous IMU sample: timestamps, stored measurements and kinematics, and attitude.
     *
     * @param estimator    estimator to be checked.
     * @param measurements provided measurements.
     * @param c            attitude of the estimator before the measurements.
     * @param timeSeconds  timestamp of the IMU sample expressed in seconds.
     */
    private static void assertMeasurementsAfterKinematics(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator,
            final Collection<GNSSMeasurement> measurements, final CoordinateTransformation c,
            final double timeSeconds) {
        assertFalse(estimator.isRunning());
        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), timestamp);
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(new BodyKinematics(), estimator.getKinematics());
        assertEquals(new BodyKinematics(), estimator.getCorrectedKinematics());

        // attitude is kept
        assertTrue(c.equals(estimator.getCoordinateTransformation(), ABSOLUTE_ERROR));
        final var c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertTrue(c.equals(c2, ABSOLUTE_ERROR));
    }

    /**
     * Checks the behaviour of body kinematics updates once the solution has been initialised with GNSS
     * measurements: the first sample only sets the time reference (no navigation, no Kalman epoch), a
     * repeated or older timestamp is ignored, and the second one is integrated with the ECEF inertial
     * navigation equations over the elapsed interval (position, velocity and attitude).
     *
     * @param truth       true frame at the time of the measurements. Its attitude is the initial
     *                    attitude of the estimator.
     * @param kinematics  kinematics provided on both samples.
     * @param timeSeconds timestamp of the measurements and of the first sample, and interval up to the
     *                    second one, expressed in seconds.
     * @return solution of the estimator before and after integrating the second sample.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    private Integration assertUpdateBodyKinematicsIntegratesSample(
            final ECEFFrame truth, final BodyKinematics kinematics, final double timeSeconds)
            throws LockedException, NotReadyException, INSGNSSException, InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        final var estimator = new INSGNSSTightlyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), truth.getCoordinateTransformation(), this);

        reset();
        assertListenerCounts(0, 0, 0);

        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));

        // update measurement
        final var measurements = generateMeasurements(timeSeconds, truth);
        assertTrue(estimator.updateMeasurements(measurements, timeSeconds));

        final var state1 = assertSolutionEquals(truth, estimator);
        final var initialFrame = currentFrame(estimator);
        assertNull(estimator.getLastKinematicsTimestamp());

        // first sample only sets the time reference: no navigation and no Kalman filter epoch
        assertTrue(estimator.updateBodyKinematics(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastKinematicsTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastKinematicsTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
        assertEquals(kinematics, estimator.getKinematics());
        // first Kalman epoch estimates no bias, so corrected kinematics equal provided ones
        assertEquals(kinematics, estimator.getCorrectedKinematics());
        assertEquals(state1, estimator.getState());
        assertEquals(state1.getGNSSEstimation(), estimator.getEstimation());

        // a sample with the same or an older timestamp makes no action
        assertFalse(estimator.updateBodyKinematics(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));
        assertFalse(estimator.updateBodyKinematics(kinematics, HALF * timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(state1, estimator.getState());

        // a sample with a new timestamp is integrated over the elapsed interval
        assertTrue(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(2.0 * timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(kinematics, estimator.getKinematics());
        assertEquals(kinematics, estimator.getCorrectedKinematics());

        final var expected = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame, List.of(kinematics),
                timeSeconds);
        final var result = currentFrame(estimator);
        assertFrameEquals(expected, result);

        // estimation and state hold the INS solution, and the covariance is untouched without a Kalman
        // filter epoch
        final var state3 = estimator.getState();
        assertEquals(estimator.getEstimation(), state3.getGNSSEstimation());
        assertFrameEquals(result, state3.getFrame());
        assertEquals(state1.getCovariance(), state3.getCovariance());

        // Kalman filter epochs are only notified when measurements are provided
        assertListenerCounts(1, 2, 1);

        return new Integration(initialFrame, result);
    }

    /**
     * Asserts that a frame is the result of a free fall from an initial frame: displacement
     * v0 * t + g * t^2 / 2 and velocity v0 + g * t, where g is the gravity at the initial position.
     *
     * @param initialFrame frame at the start of the fall.
     * @param result       frame at the end of the fall.
     * @param duration     duration of the fall expressed in seconds.
     */
    private static void assertFreeFall(final ECEFFrame initialFrame, final ECEFFrame result, final double duration) {
        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(initialFrame.getX(),
                initialFrame.getY(), initialFrame.getZ());
        final var factor = HALF * duration * duration;
        final var displacementError = norm(
                result.getX() - initialFrame.getX() - initialFrame.getVx() * duration - gravity.getGx() * factor,
                result.getY() - initialFrame.getY() - initialFrame.getVy() * duration - gravity.getGy() * factor,
                result.getZ() - initialFrame.getZ() - initialFrame.getVz() * duration - gravity.getGz() * factor);
        final var velocityError = norm(
                result.getVx() - initialFrame.getVx() - gravity.getGx() * duration,
                result.getVy() - initialFrame.getVy() - gravity.getGy() * duration,
                result.getVz() - initialFrame.getVz() - gravity.getGz() * duration);

        assertEquals(0.0, displacementError, FREE_FALL_POSITION_TOLERANCE);
        assertEquals(0.0, velocityError, FREE_FALL_VELOCITY_TOLERANCE);
    }

    /**
     * Angle rotated by the body between two frames, with respect to the ECEF frame.
     *
     * @param initialFrame frame before the rotation.
     * @param result       frame after the rotation.
     * @return rotated angle expressed in radians.
     */
    private static double rotationAngle(final ECEFFrame initialFrame, final ECEFFrame result) {
        final var initial = initialFrame.getCoordinateTransformationMatrix();
        final var rotated = result.getCoordinateTransformationMatrix();

        // trace of initial' * rotated, which is the sum of the element-wise products
        var trace = 0.0;
        for (var i = 0; i < initial.getRows(); i++) {
            for (var j = 0; j < initial.getColumns(); j++) {
                trace += initial.getElementAt(i, j) * rotated.getElementAt(i, j);
            }
        }
        return Math.acos(Math.min(1.0, HALF * (trace - 1.0)));
    }

    /**
     * Creates the deterministic Kalman filter configuration used by the trajectory tests (tactical
     * grade values of Groves' {@code INS_GNSS_Demo_10}).
     *
     * @return Kalman filter configuration.
     */
    private static INSTightlyCoupledKalmanConfig createKalmanConfig() {
        return new INSTightlyCoupledKalmanConfig(GYRO_NOISE_PSD, ACCELEROMETER_NOISE_PSD, ACCELEROMETER_BIAS_PSD,
                GYRO_BIAS_PSD, CLOCK_FREQUENCY_PSD, CLOCK_PHASE_PSD, PSEUDO_RANGE_SD, RANGE_RATE_SD);
    }

    /**
     * Creates the deterministic initial uncertainties used by the trajectory tests (values of Groves'
     * {@code INS_GNSS_Demo_10}).
     *
     * @return Kalman filter initializer configuration.
     */
    private static INSTightlyCoupledKalmanInitializerConfig createInitConfig() {
        return new INSTightlyCoupledKalmanInitializerConfig(INITIAL_ATTITUDE_UNCERTAINTY,
                INITIAL_VELOCITY_UNCERTAINTY, INITIAL_POSITION_UNCERTAINTY, INITIAL_ACCELEROMETER_BIAS_UNCERTAINTY,
                INITIAL_GYRO_BIAS_UNCERTAINTY, INITIAL_CLOCK_OFFSET_UNCERTAINTY, INITIAL_CLOCK_DRIFT_UNCERTAINTY);
    }

    /**
     * Creates the errors of a tactical grade IMU (values of Groves' {@code INS_GNSS_Demo_10}).
     *
     * @return IMU errors.
     * @throws WrongSizeException never thrown.
     */
    private static IMUErrors createImuErrors() throws WrongSizeException {
        final var accelerometerBiases = new double[ACCELEROMETER_BIASES_MICRO_G.length];
        final var gyroBiases = new double[GYRO_BIASES_DEG_PER_HOUR.length];
        for (var i = 0; i < accelerometerBiases.length; i++) {
            accelerometerBiases[i] = ACCELEROMETER_BIASES_MICRO_G[i] * MICRO_G;
            gyroBiases[i] = GYRO_BIASES_DEG_PER_HOUR[i] * DEG_PER_HOUR;
        }

        final var ma = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        ma.fromArray(ACCELEROMETER_SCALE_AND_CROSS_COUPLING, false);
        final var mg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        mg.fromArray(GYRO_SCALE_AND_CROSS_COUPLING, false);
        final var gg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        gg.fromArray(GYRO_G_DEPENDENT_BIASES_DEG_PER_HOUR_PER_G, false);
        gg.multiplyByScalar(DEG_PER_HOUR / STANDARD_GRAVITY);

        return new IMUErrors(accelerometerBiases, gyroBiases, ma, mg, gg, ACCELEROMETER_NOISE_ROOT_PSD,
                GYRO_NOISE_ROOT_PSD, 0.0, 0.0);
    }

    /**
     * Indicates whether the position error of a Kalman filter state lies, on every axis, within
     * {@link #SIGMA_FACTOR} standard deviations of its estimated covariance.
     *
     * @param truth true frame.
     * @param state Kalman filter state.
     * @return true if the position error is consistent with the covariance, false otherwise.
     */
    private static boolean isPositionWithinCovariance(
            final ECEFFrame truth, final INSTightlyCoupledKalmanState state) {
        final var covariance = state.getCovariance();
        final var errors = new double[]{
                state.getX() - truth.getX(), state.getY() - truth.getY(), state.getZ() - truth.getZ()};
        for (var i = 0; i < POSITION_COMPONENTS; i++) {
            final var index = POSITION_STATE_INDEX + i;
            if (Math.abs(errors[i]) > SIGMA_FACTOR * Math.sqrt(covariance.getElementAt(index, index))) {
                return false;
            }
        }
        return true;
    }

    /**
     * Solution of an estimator before and after integrating an IMU sample.
     *
     * @param initialFrame solution before integrating the sample.
     * @param result       solution after integrating the sample.
     */
    private record Integration(ECEFFrame initialFrame, ECEFFrame result) {
    }
}
