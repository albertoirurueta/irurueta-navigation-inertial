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
import com.irurueta.geometry.InvalidRotationMatrixException;
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
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.RadiiOfCurvatureEstimator;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class INSLooselyCoupledKalmanFilteredEstimatorTest implements INSLooselyCoupledKalmanFilteredEstimatorListener {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double MIN_EPOCH_INTERVAL = 1e-5;
    private static final double MAX_EPOCH_INTERVAL = 1.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_USER_HEIGHT = -50.0;
    private static final double MAX_USER_HEIGHT = 50.0;

    private static final double MIN_USER_VELOCITY_VALUE = -2.0;
    private static final double MAX_USER_VELOCITY_VALUE = 2.0;

    private static final double MIN_TIME = 0.0;
    private static final double MAX_TIME = 1.0;

    private static final double MIN_DEGREES_PER_SECOND = -10.0;
    private static final double MAX_DEGREES_PER_SECOND = 10.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    /**
     * Yaw rate of the yawing trajectory, expressed in radians per second.
     */
    private static final double YAW_RATE = 0.1;

    /**
     * Duration of the yawing and northbound trajectories, expressed in seconds.
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
     * Duration of the free fall, expressed in seconds.
     */
    private static final double FREE_FALL_DURATION = 1.0;

    /**
     * Factor of the displacement of a uniformly accelerated body starting at rest (g * t^2 / 2).
     */
    private static final double HALF = 0.5;

    /**
     * Tolerance, in radians, for the yaw integrated over the yawing trajectory.
     * The kinematics generated by {@code ECEFKinematicsEstimator} are the exact inverse of the
     * attitude update of {@code ECEFInertialNavigator} up to rounding errors, which after 1000 samples
     * remain several orders of magnitude below this value; 1e-6 rad (0.2 arc-seconds) is far below
     * the 1 rad that is lost when rotation is not integrated.
     */
    private static final double YAW_TOLERANCE = 1e-6;

    /**
     * Tolerance, in meters, for the position reached against the analytic truth of a trajectory.
     * The navigator integrates position with the trapezoidal rule (eq. 5.38), whose error for the
     * centripetal acceleration of a 10 m/s motion over the Earth (1.6e-5 m/s^2) sampled at 100 Hz is
     * well below a micrometer; 1e-3 m leaves margin for the truth integration while being five
     * orders of magnitude below the 100 m travelled.
     */
    private static final double TRUTH_POSITION_TOLERANCE = 1e-3;

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
     * Tolerance, in meters, for the displacement of a 1 s free fall against g * t^2 / 2. The
     * constant gravity approximation neglects the Coriolis acceleration (2 * 7.29e-5 rad/s * 9.8 m/s
     * = 1.4e-3 m/s^2 at most, i.e. 2.4e-4 m after 1 s) and the gravity gradient (3e-6 s^-2 * 4.9 m);
     * 1e-2 m bounds both with margin while being 500 times smaller than the 4.9 m fallen.
     */
    private static final double FREE_FALL_POSITION_TOLERANCE = 1e-2;

    /**
     * Tolerance, in meters per second, for the velocity gained in a 1 s free fall against g * t.
     * The neglected Coriolis term integrates to 7.29e-5 rad/s * 9.8 m/s^2 * (1 s)^2 = 7.1e-4 m/s
     * at most, hence 1e-3 m/s.
     */
    private static final double FREE_FALL_VELOCITY_TOLERANCE = 1e-3;

    /**
     * Gyro noise PSD of the Kalman filter, (0.02 deg/sqrt(hr))^2 expressed in rad^2/s, as used by the
     * tactical grade configuration of Groves' {@code INS_GNSS_Demo_3}.
     */
    private static final double GYRO_NOISE_PSD = Math.pow(Math.toRadians(0.02) / 60.0, 2.0);

    /**
     * Accelerometer noise PSD of the Kalman filter, (200 micro-g/sqrt(Hz))^2 expressed in m^2/s^3,
     * as used by the tactical grade configuration of Groves' {@code INS_GNSS_Demo_3}.
     */
    private static final double ACCELEROMETER_NOISE_PSD = Math.pow(200.0e-6 * 9.80665, 2.0);

    /**
     * Accelerometer bias random walk PSD of the Kalman filter, expressed in m^2/s^5 (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double ACCELEROMETER_BIAS_PSD = 1.0e-7;

    /**
     * Gyro bias random walk PSD of the Kalman filter, expressed in rad^2/s^3 (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double GYRO_BIAS_PSD = 2.0e-12;

    /**
     * Standard deviation per axis of the position fixes, expressed in meters (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double FIX_POSITION_SD = 2.5;

    /**
     * Standard deviation per axis of the velocity fixes, expressed in meters per second (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double FIX_VELOCITY_SD = 0.1;

    /**
     * Initial attitude uncertainty per axis, 1 degree expressed in radians (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double INITIAL_ATTITUDE_UNCERTAINTY = Math.toRadians(1.0);

    /**
     * Initial velocity uncertainty per axis, expressed in meters per second (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double INITIAL_VELOCITY_UNCERTAINTY = 0.1;

    /**
     * Initial position uncertainty per axis, expressed in meters (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double INITIAL_POSITION_UNCERTAINTY = 10.0;

    /**
     * Initial accelerometer bias uncertainty per axis, 1000 micro-g expressed in m/s^2 (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double INITIAL_ACCELEROMETER_BIAS_UNCERTAINTY = 1000.0e-6 * 9.80665;

    /**
     * Initial gyro bias uncertainty per axis, 10 deg/hr expressed in rad/s (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double INITIAL_GYRO_BIAS_UNCERTAINTY = Math.toRadians(10.0) / 3600.0;

    /**
     * Seed of the pseudo-random generators, so that every run uses exactly the same inputs.
     */
    private static final long SEED = 36L;

    /**
     * Tolerance, in meters, for the displacement of a free fall of up to 1 s starting with a speed of
     * up to 3.5 m/s (2 m/s per axis) against v0 * t + g * t^2 / 2. The neglected Coriolis acceleration
     * is at most 2 * 7.29e-5 rad/s * (3.5 + 9.8) m/s = 1.9e-3 m/s^2, i.e. 1e-3 m after 1 s, and the
     * gravity gradient over the 8.4 m travelled adds 3e-6 s^-2 * 8.4 m = 2.5e-5 m/s^2; hence 1e-2 m.
     */
    private static final double UPDATE_FREE_FALL_POSITION_TOLERANCE = 1e-2;

    /**
     * Tolerance, in meters per second, for the velocity of a free fall of up to 1 s starting with a
     * speed of up to 3.5 m/s against v0 + g * t: the neglected Coriolis acceleration (1.9e-3 m/s^2 at
     * most, see {@link #UPDATE_FREE_FALL_POSITION_TOLERANCE}) integrates to 1.9e-3 m/s; hence 5e-3 m/s.
     */
    private static final double UPDATE_FREE_FALL_VELOCITY_TOLERANCE = 5e-3;

    /**
     * Distance, in meters, by which the INS is initialised north of the truth to check that a fix
     * corrects the solution towards the measurement. It equals the initial position uncertainty.
     */
    private static final double NORTH_OFFSET = 10.0;

    /**
     * Duration, in seconds, of the stationary trajectory integrated before providing the fix.
     */
    private static final double FIX_TEST_DURATION = 1.0;

    /**
     * Epoch interval, in seconds, used to check the epoch gate. It is exactly representable in
     * binary floating point, as are the fix timestamps, so that intervals are exact.
     */
    private static final double GATE_EPOCH_INTERVAL = 0.5;

    /**
     * Timestamp, in seconds, of a fix arriving exactly two epoch intervals after the first one.
     */
    private static final double GATE_THIRD_FIX_TIMESTAMP = 1.0;

    /**
     * Timestamp, in seconds, of a fix arriving only half an epoch interval after the previous one.
     */
    private static final double GATE_EARLY_FIX_TIMESTAMP = 1.25;

    /**
     * Duration, in seconds, of the closed-loop trajectory.
     */
    private static final double CLOSED_LOOP_DURATION = 60.0;

    /**
     * Number of IMU samples between consecutive fixes of the closed-loop test (1 s at 100 Hz).
     */
    private static final int SAMPLES_PER_FIX = 100;

    /**
     * Time, in seconds, after which the closed-loop filter is considered converged: ten fixes, enough
     * for the position covariance to settle from the 10 m initial uncertainty.
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
     * Yaw error, 2 degrees expressed in radians, of the INS at the start of the closed-loop test with an
     * initial attitude error. It is twice the initial attitude uncertainty given to the filter, so
     * that the attitude corrections applied by the closed loop are clearly larger than rounding.
     */
    private static final double INITIAL_YAW_ERROR = Math.toRadians(2.0);

    /**
     * Index of the first position element within the Kalman filter state (attitude, velocity,
     * position, accelerometer bias and gyro bias, three elements each).
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
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double[] ACCELEROMETER_BIASES_MICRO_G = {900.0, -1300.0, 800.0};

    /**
     * Gyro biases of a tactical grade IMU, expressed in degrees per hour (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double[] GYRO_BIASES_DEG_PER_HOUR = {-9.0, 13.0, -8.0};

    /**
     * Accelerometer scale factor and cross coupling errors of a tactical grade IMU, in row order
     * (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double[] ACCELEROMETER_SCALE_AND_CROSS_COUPLING = {
            500e-6, -300e-6, 200e-6,
            -150e-6, -600e-6, 250e-6,
            -250e-6, 100e-6, 450e-6
    };

    /**
     * Gyro scale factor and cross coupling errors of a tactical grade IMU, in row order (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double[] GYRO_SCALE_AND_CROSS_COUPLING = {
            400e-6, -300e-6, 250e-6,
            0.0, -300e-6, -150e-6,
            0.0, 0.0, -350e-6
    };

    /**
     * Gyro g-dependent biases of a tactical grade IMU, in row order and expressed in degrees per hour
     * per g (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double[] GYRO_G_DEPENDENT_BIASES_DEG_PER_HOUR_PER_G = {
            0.9, -1.1, -0.6,
            -0.5, 1.9, -1.6,
            0.3, 1.1, -1.3
    };

    /**
     * Accelerometer noise root PSD of a tactical grade IMU, 100 micro-g/sqrt(Hz) expressed in
     * m * s^-1.5 (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double ACCELEROMETER_NOISE_ROOT_PSD = 100.0 * MICRO_G;

    /**
     * Gyro noise root PSD of a tactical grade IMU, 0.01 deg/sqrt(hr) expressed in rad * s^-0.5
     * (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double GYRO_NOISE_ROOT_PSD = Math.toRadians(0.01) / 60.0;

    private int updateStart;
    private int updateEnd;
    private int propagateStart;
    private int propagateEnd;
    private int reset;

    @Test
    void testConstructor() throws InvalidSourceAndDestinationFrameTypeException {

        // test constructor 1
        var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        var epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
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
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        var kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 3
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval));

        // test constructor 4
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 5
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval));

        // test constructor 6
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 7
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, this));

        // test constructor 8
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, this));

        // test constructor 9
        epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 10
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        final var wrongEpochIntervalTime = new Time(-epochInterval, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime));

        // test constructor 11
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, this));

        // test constructor 12
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, this));

        // test constructor 13
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

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

        final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        estimator = new INSLooselyCoupledKalmanFilteredEstimator(frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        var frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 14
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // test constructor 15
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                frame));

        // test constructor 16
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 17
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, frame));

        // test constructor 18
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // test constructor 19
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                frame, this));

        // test constructor 20
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, frame, this));

        // test constructor 21
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, frame));

        // test constructor 22
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, frame));

        // test constructor 23
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, frame, this));

        // test constructor 24
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, frame, this));

        // test constructor 25
        final var initialConfig = generateInitConfig();
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        INSLooselyCoupledKalmanInitializerConfig initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 26
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 27
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                initialConfig));

        // test constructor 28
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 29
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, initialConfig));

        // test constructor 30
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 31
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, this));

        // test constructor 32
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, 
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, initialConfig, this));

        // test constructor 33
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig));

        // test constructor 34
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, initialConfig));

        // test constructor 35
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, this));

        // test constructor 36
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, initialConfig, this));

        // test constructor 37
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 38
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // test constructor 39
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                initialConfig, frame));

        // test constructor 40
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 41
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, initialConfig, frame));

        // test constructor 42
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // test constructor 43
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, frame, this));

        // test constructor 44
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, frame,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, initialConfig, frame, this));

        // test constructor 45
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, frame));

        // test constructor 46
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, initialConfig, frame));

        // test constructor 47
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, frame, this));

        // test constructor 48
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, frame,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, initialConfig, frame, this));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetEpochInterval() throws LockedException {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

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
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

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
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getConfig());
        assertFalse(estimator.getConfig(null));

        // set a new value
        final var config1 = generateKalmanConfig();
        estimator.setConfig(config1);

        // check
        final var config2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(config2));
        final var config3 = estimator.getConfig();

        assertEquals(config1, config2);
        assertEquals(config1, config3);
    }

    @Test
    void testGetSetFrame() throws LockedException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

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

        final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));

        // set a new value
        estimator.setFrame(frame);

        // check
        assertEquals(frame, estimator.getFrame());

        final var frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
    }

    @Test
    void testGetSetInitialConfig() throws LockedException {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getInitialConfig());
        assertFalse(estimator.getInitialConfig(null));

        // set a new value
        final var config1 = generateInitConfig();
        estimator.setInitialConfig(config1);

        // check
        final var config2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(config2));
        final var config3 = estimator.getInitialConfig();

        assertEquals(config1, config2);
        assertSame(config1, config3);
    }

    /**
     * Checks that a sample with zero specific force and zero angular rate is integrated as a free
     * fall, and that the first sample only initialises the estimator.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSException                                  if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     */
    @Test
    void testUpdateWithZeroSpecificForceAndAngularRate() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, INSException, InertialNavigatorException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var frame = createFrame(randomizer);
        final var initialFrame = new ECEFFrame(frame);

        final var kinematics = new BodyKinematics();
        final var result = assertUpdateIntegratesSample(frame, kinematics, timeSeconds);

        assertFreeFall(initialFrame, result, timeSeconds, UPDATE_FREE_FALL_POSITION_TOLERANCE,
                UPDATE_FREE_FALL_VELOCITY_TOLERANCE);
    }

    /**
     * Checks that a sample with specific force and zero angular rate is integrated with the ECEF
     * inertial navigation equations, and that the first sample only initialises the estimator.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSException                                  if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     */
    @Test
    void testUpdateWithSpecificForceAndZeroAngularRate() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, INSException, InertialNavigatorException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var frame = createFrame(randomizer);
        final var initialFrame = new ECEFFrame(frame);

        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(frame.getX(), frame.getY(),
                frame.getZ());
        final var kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                0.0, 0.0, 0.0);
        final var result = assertUpdateIntegratesSample(frame, kinematics, timeSeconds);

        // provided specific force makes the body move differently from a free fall
        final var freeFall = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame,
                List.of(new BodyKinematics()), timeSeconds);
        assertTrue(velocityError(freeFall, result) > UPDATE_FREE_FALL_VELOCITY_TOLERANCE);
    }

    /**
     * Checks that a sample with zero specific force and angular rate is integrated as a free fall
     * while the attitude is rotated, and that the first sample only initialises the estimator.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSException                                  if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     */
    @Test
    void testUpdateWithZeroSpecificForceAndRotationOnly() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, INSException, InertialNavigatorException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var frame = createFrame(randomizer);
        final var initialFrame = new ECEFFrame(frame);

        final var angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var kinematics = new BodyKinematics(0.0, 0.0, 0.0, angularRateX, angularRateY, angularRateZ);
        final var result = assertUpdateIntegratesSample(frame, kinematics, timeSeconds);

        assertFreeFall(initialFrame, result, timeSeconds, UPDATE_FREE_FALL_POSITION_TOLERANCE,
                UPDATE_FREE_FALL_VELOCITY_TOLERANCE);

        // attitude has been rotated (the oracle comparison has already checked its value)
        assertFalse(initialFrame.getCoordinateTransformationMatrix().equals(
                result.getCoordinateTransformationMatrix(), CHAIN_ROTATION_TOLERANCE));
    }

    /**
     * Checks that a sample with both specific force and angular rate is integrated with the ECEF
     * inertial navigation equations, and that the first sample only initialises the estimator.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSException                                  if estimation fails.
     * @throws InertialNavigatorException                    if the oracle navigation fails.
     */
    @Test
    void testUpdateWithSpecificForceAndRotation() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, INSException, InertialNavigatorException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var frame = createFrame(randomizer);
        final var initialFrame = new ECEFFrame(frame);

        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(frame.getX(), frame.getY(),
                frame.getZ());
        final var angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                angularRateX, angularRateY, angularRateZ);
        final var result = assertUpdateIntegratesSample(frame, kinematics, timeSeconds);

        // provided specific force makes the body move differently from a free fall
        final var freeFall = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame,
                List.of(new BodyKinematics()), timeSeconds);
        assertTrue(velocityError(freeFall, result) > UPDATE_FREE_FALL_VELOCITY_TOLERANCE);

        // attitude has been rotated (the oracle comparison has already checked its value)
        assertFalse(initialFrame.getCoordinateTransformationMatrix().equals(
                result.getCoordinateTransformationMatrix(), CHAIN_ROTATION_TOLERANCE));
    }

    @Test
    void testUpdateWhenNotReady() {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        assertFalse(estimator.isUpdateReady());

        assertThrows(NotReadyException.class, () -> estimator.update(new BodyKinematics(), 0.0));
    }

    /**
     * Checks that propagate only initialises the Kalman filter state and keeps it synchronised with
     * the frame: it never runs a Kalman filter epoch, never notifies the listener and returns false.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSException                                  if estimation fails.
     */
    @SuppressWarnings("deprecation")
    @Test
    void testPropagate() throws InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            INSException {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var frame = createFrame(randomizer);
        final var initialFrame = new ECEFFrame(frame);

        final var initConfig = createInitConfig();
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), initConfig, frame,
                this);

        reset();
        assertNull(estimator.getState());
        assertTrue(estimator.isPropagateReady());

        // propagate before any sample initialises the state, but runs no Kalman filter epoch
        assertFalse(estimator.propagate(timeSeconds));

        assertFalse(estimator.isRunning());
        assertNull(estimator.getLastStateTimestamp());
        assertNull(estimator.getLastKinematicsTimestamp());
        final var state1 = estimator.getState();
        assertNotNull(state1);
        assertFrameEquals(initialFrame, state1.getFrame());
        assertEquals(initialFrame, estimator.getFrame());
        assertEquals(INSLooselyCoupledKalmanInitializer.initialize(initConfig), state1.getCovariance());

        // propagate again makes no change
        assertFalse(estimator.propagate(new Time(2.0 * timeSeconds, TimeUnit.SECOND)));
        assertEquals(state1, estimator.getState());

        // after integrating samples, state holds the INS solution and propagate makes no change
        final var kinematics = new BodyKinematics();
        assertTrue(estimator.update(kinematics, timeSeconds));
        assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

        final var state2 = estimator.getState();
        assertNotEquals(state1, state2);
        assertFrameEquals(estimator.getFrame(), state2.getFrame());

        assertFalse(estimator.propagate(MAX_TIME + 2.0 * timeSeconds));

        assertEquals(state2, estimator.getState());
        assertEquals(state1.getCovariance(), state2.getCovariance());
        assertNull(estimator.getLastStateTimestamp());
        assertEquals(2.0 * timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);

        // only updates are notified
        assertEquals(2, updateStart);
        assertEquals(2, updateEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);
    }

    @SuppressWarnings("deprecation")
    @Test
    void testPropagateWhenNotReady() {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        assertFalse(estimator.isPropagateReady());

        assertThrows(NotReadyException.class, () -> estimator.propagate(0.0));
    }

    @Test
    void testReset() throws InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            INSException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer(new Random(SEED));

        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

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

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        final var kalmanConfig = createKalmanConfig();
        final var initConfig = createInitConfig();
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame,
                this);

        reset();
        assertEquals(0, updateStart);
        assertEquals(0, updateEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);

        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertTrue(estimator.isUpdateReady());
        assertTrue(estimator.isPropagateReady());

        // update kinematics for the first time, makes no change
        final var kinematics = new BodyKinematics();
        assertTrue(estimator.update(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertNull(estimator.getLastStateTimestamp());

        // a fix at the current position runs a Kalman filter epoch without moving the solution
        assertTrue(estimator.updatePositionAndVelocity(ecefUserPosition, ecefUserVelocity, timeSeconds));

        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
        assertEquals(kinematics, estimator.getKinematics());
        assertNotNull(estimator.getCorrectedKinematics());

        final var state1 = estimator.getState();
        assertNotNull(state1);

        assertTrue(estimator.getFrame().equals(state1.getFrame(), ABSOLUTE_ERROR));

        final var estimatedPosition1 = state1.getEcefPosition();
        final var estimatedVelocity1 = state1.getEcefVelocity();
        final var estimatedC1 = state1.getC();

        assertTrue(estimatedPosition1.equals(ecefUserPosition, ABSOLUTE_ERROR));
        assertTrue(estimatedVelocity1.equals(ecefUserVelocity, ABSOLUTE_ERROR));
        assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

        // reset
        assertEquals(0, reset);

        estimator.reset();

        assertEquals(1, reset);
        assertNull(estimator.getState());
        assertNull(estimator.getLastStateTimestamp());
        assertNull(estimator.getLastKinematicsTimestamp());
        assertNull(estimator.getFrame());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.isRunning());

        assertFalse(estimator.isUpdateReady());

        // set frame
        estimator.setFrame(frame);

        assertTrue(estimator.isUpdateReady());

        // update again with the same timestamp now it does make an action
        assertTrue(estimator.update(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));

        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertNull(estimator.getLastStateTimestamp());
        final var state2 = estimator.getState();

        // state is initialised again from the frame and the initial covariance
        assertTrue(state2.getFrame().equals(frame, ABSOLUTE_ERROR));
        assertEquals(INSLooselyCoupledKalmanInitializer.initialize(initConfig), state2.getCovariance());

        // a fix with the same timestamp is accepted again after the reset
        assertTrue(estimator.updatePositionAndVelocity(ecefUserPosition, ecefUserVelocity, timeSeconds));
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);

        assertEquals(2, updateStart);
        assertEquals(2, updateEnd);
        assertEquals(2, propagateStart);
        assertEquals(2, propagateEnd);
    }

    /**
     * Checks that the angular rate of every IMU sample is integrated: after a 10 s trajectory yawing at
     * 0.1 rad/s the estimated yaw must be 1 rad.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testRotationIsIntegrated() throws LockedException, NotReadyException, INSException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.yawingStationary(YAW_RATE, TRAJECTORY_DURATION);
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                trajectory.getInitialFrame());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);

        feed(estimator, trajectory);

        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(estimator.getFrame());
        assertEquals(EXPECTED_FINAL_YAW, nedFrame.getCoordinateTransformation().getYawEulerAngle(), YAW_TOLERANCE);
    }

    /**
     * Checks that the specific force of every IMU sample is integrated: after moving north at 10 m/s
     * for 10 s the estimated position and velocity must be the true ones.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testTranslationIsIntegrated() throws LockedException, NotReadyException, INSException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.northAtConstantSpeed(NORTH_SPEED,
                TRAJECTORY_DURATION);
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                trajectory.getInitialFrame());

        feed(estimator, trajectory);

        final var expected = trajectory.getFinalFrame();
        final var result = estimator.getFrame();
        assertEquals(0.0, positionError(expected, result), TRUTH_POSITION_TOLERANCE);
        assertEquals(0.0, velocityError(expected, result), CHAIN_VELOCITY_TOLERANCE);
    }

    /**
     * Checks that without position and velocity fixes the estimator is a pure inertial navigator: its
     * frame must equal the one obtained by chaining the ECEF inertial navigator over the same samples.
     *
     * @throws LockedException            if estimator is locked.
     * @throws NotReadyException          if estimator is not ready.
     * @throws INSException               if estimation fails.
     * @throws InertialNavigatorException if the oracle navigation fails.
     */
    @Test
    void testMatchesChainedNavigatorWithoutFix() throws LockedException, NotReadyException, INSException,
            InertialNavigatorException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.turningAndAccelerating(TRAJECTORY_DURATION);
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(new INSLooselyCoupledKalmanConfig(),
                trajectory.getInitialFrame());

        feed(estimator, trajectory);

        final var expected = KalmanFilteredEstimatorTrajectory.chainNavigator(trajectory.getInitialFrame(),
                trajectory.getKinematics(), trajectory.getSampleInterval());
        assertFrameEquals(expected, estimator.getFrame());
    }

    /**
     * Checks that IMU samples arriving within a Kalman epoch interval are all integrated: with a 1 s
     * epoch interval and 100 Hz samples of a body that accelerates and then decelerates, the frame
     * must equal the per-sample integration, not a single step using only the last sample.
     *
     * @throws LockedException            if estimator is locked.
     * @throws NotReadyException          if estimator is not ready.
     * @throws INSException               if estimation fails.
     * @throws InertialNavigatorException if the oracle navigation fails.
     */
    @Test
    void testEveryImuSampleCountsWithinEpoch() throws LockedException, NotReadyException, INSException,
            InertialNavigatorException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.accelerateThenDecelerate(HALF_EPOCH_DURATION,
                HALF_EPOCH_DURATION);
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                ONE_SECOND_EPOCH_INTERVAL, createInitConfig(), trajectory.getInitialFrame());

        final var integrated = feed(estimator, trajectory);

        final var kinematics = trajectory.getKinematics();
        final var expected = KalmanFilteredEstimatorTrajectory.chainNavigator(trajectory.getInitialFrame(),
                kinematics, trajectory.getSampleInterval());
        final var result = estimator.getFrame();
        assertFrameEquals(expected, result);

        // a single step over the whole epoch using only the last sample gives a different solution
        final var singleStep = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(ONE_SECOND_EPOCH_INTERVAL, trajectory.getInitialFrame(),
                kinematics.getLast(), singleStep);
        assertTrue(positionError(singleStep, result) > TRUTH_POSITION_TOLERANCE);

        // the initial sample plus one per interval were all accepted
        assertEquals(kinematics.size() + 1, integrated);
    }

    /**
     * Checks that with zero specific force the body falls freely: after 1 s from rest the displacement
     * must be g * t^2 / 2 and the velocity g * t along the local gravity vector.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testFreeFall() throws LockedException, NotReadyException, INSException {
        final var initialFrame = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, FREE_FALL_DURATION)
                .getInitialFrame();
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                new ECEFFrame(initialFrame));

        final var numSamples = Math.round(FREE_FALL_DURATION / KalmanFilteredEstimatorTrajectory.SAMPLE_INTERVAL);
        final var freeFallKinematics = new BodyKinematics();
        for (var i = 0; i <= numSamples; i++) {
            estimator.update(freeFallKinematics, i * KalmanFilteredEstimatorTrajectory.SAMPLE_INTERVAL);
        }

        assertFreeFall(initialFrame, estimator.getFrame(), FREE_FALL_DURATION, FREE_FALL_POSITION_TOLERANCE,
                FREE_FALL_VELOCITY_TOLERANCE);
    }

    /**
     * Checks that a position and velocity fix corrects the INS solution towards the measurement: with
     * the INS initialised 10 m north of the truth, a fix at the true position reduces the position
     * error, moves the solution towards the fix and reduces the position covariance.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testFixCorrectsTowardsMeasurement() throws LockedException, NotReadyException, INSException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, FIX_TEST_DURATION);
        final var truth = trajectory.getFinalFrame();

        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                createFrameNorthOf(trajectory.getInitialFrame()), this);
        reset();

        final var numSamples = feed(estimator, trajectory);

        final var before = estimator.getFrame();
        final var covarianceBefore = positionCovarianceTrace(estimator.getState());
        final var errorBefore = positionError(truth, before);
        assertEquals(NORTH_OFFSET, errorBefore, TRUTH_POSITION_TOLERANCE);
        assertNull(estimator.getLastStateTimestamp());
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);

        assertTrue(estimator.updatePositionAndVelocity(truth.getECEFPosition(), truth.getECEFVelocity(),
                FIX_TEST_DURATION));

        final var after = estimator.getFrame();
        final var errorAfter = positionError(truth, after);
        assertTrue(errorAfter < errorBefore);

        // correction points towards the fix
        final var projection = (after.getX() - before.getX()) * (truth.getX() - before.getX())
                + (after.getY() - before.getY()) * (truth.getY() - before.getY())
                + (after.getZ() - before.getZ()) * (truth.getZ() - before.getZ());
        assertTrue(projection > 0.0);

        // position uncertainty decreases, and state holds the corrected INS solution
        final var state = estimator.getState();
        assertTrue(positionCovarianceTrace(state) < covarianceBefore);
        assertFrameEquals(after, state.getFrame());

        assertFalse(estimator.isRunning());
        assertEquals(FIX_TEST_DURATION, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(FIX_TEST_DURATION, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        assertEquals(FIX_TEST_DURATION, estimator.getLastKinematicsTimestamp(), ABSOLUTE_ERROR);
        assertEquals(numSamples, updateStart);
        assertEquals(numSamples, updateEnd);
        assertEquals(1, propagateStart);
        assertEquals(1, propagateEnd);
    }

    /**
     * Checks the epoch gate of position and velocity fixes: fixes arriving exactly one epoch interval
     * apart are accepted, while fixes arriving earlier, or repeating a timestamp, are ignored.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testEpochGateAcceptsExactInterval() throws LockedException, NotReadyException, INSException {
        final var frame = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, FIX_TEST_DURATION)
                .getInitialFrame();
        final var position = frame.getECEFPosition();
        final var velocity = frame.getECEFVelocity();

        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                GATE_EPOCH_INTERVAL, createInitConfig(), frame, this);
        reset();

        assertTrue(estimator.updatePositionAndVelocity(position, velocity, 0.0));
        assertEquals(0.0, estimator.getLastStateTimestamp(), 0.0);

        assertTrue(estimator.updatePositionAndVelocity(position.getX(), position.getY(), position.getZ(),
                velocity.getVx(), velocity.getVy(), velocity.getVz(), GATE_EPOCH_INTERVAL));
        assertEquals(GATE_EPOCH_INTERVAL, estimator.getLastStateTimestamp(), 0.0);

        assertTrue(estimator.updatePositionAndVelocity(position, velocity,
                new Time(GATE_THIRD_FIX_TIMESTAMP, TimeUnit.SECOND)));
        assertEquals(GATE_THIRD_FIX_TIMESTAMP, estimator.getLastStateTimestamp(), 0.0);

        final var state = estimator.getState();

        // a fix arriving before a whole epoch interval has elapsed is ignored
        assertFalse(estimator.updatePositionAndVelocity(position, velocity, GATE_EARLY_FIX_TIMESTAMP));
        // a fix repeating the timestamp of the last epoch is ignored
        assertFalse(estimator.updatePositionAndVelocity(position, velocity, GATE_THIRD_FIX_TIMESTAMP));
        // a fix older than the last epoch is ignored
        assertFalse(estimator.updatePositionAndVelocity(position, velocity, GATE_EPOCH_INTERVAL));

        assertEquals(GATE_THIRD_FIX_TIMESTAMP, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(state, estimator.getState());
        assertEquals(3, propagateStart);
        assertEquals(3, propagateEnd);
        assertEquals(0, updateStart);
        assertEquals(0, updateEnd);
    }

    /**
     * Checks that a fix arriving one epoch interval after the previous epoch is accepted even when the
     * difference of both timestamps is rounded below the epoch interval (0.7 - 0.2 is lower than 0.5 in
     * floating point arithmetic).
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testEpochGateToleratesRoundingErrors() throws LockedException, NotReadyException, INSException {
        // make sure that this test exercises a difference rounded below the epoch interval
        assertTrue(0.7 - 0.2 < GATE_EPOCH_INTERVAL);

        final var frame = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, FIX_TEST_DURATION)
                .getInitialFrame();
        final var position = frame.getECEFPosition();
        final var velocity = frame.getECEFVelocity();

        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                GATE_EPOCH_INTERVAL, createInitConfig(), frame, this);
        reset();

        assertTrue(estimator.updatePositionAndVelocity(position, velocity, 0.2));
        assertTrue(estimator.updatePositionAndVelocity(position, velocity, 0.7));
        assertEquals(0.7, estimator.getLastStateTimestamp(), 0.0);

        assertFalse(INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(0.7, 0.2, GATE_EPOCH_INTERVAL));
        assertFalse(INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(1.0, 0.5, GATE_EPOCH_INTERVAL));
        assertFalse(INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(1.0, 0.5, 0.0));
        assertTrue(INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(0.69, 0.2, GATE_EPOCH_INTERVAL));
        assertTrue(INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(0.2, 0.2, GATE_EPOCH_INTERVAL));
        assertTrue(INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(0.2, 0.2, 0.0));
        assertTrue(INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(0.1, 0.2, 0.0));
    }

    /**
     * Checks that the first Kalman filter epoch propagates the covariance over the time elapsed since
     * the first IMU sample: an estimator that has integrated IMU samples before its first fix must end
     * with a different covariance (and a larger gyroscope bias uncertainty) than an estimator receiving the
     * same fix with no elapsed time.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testFirstEpochPropagatesOverElapsedTime() throws LockedException, NotReadyException, INSException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, FIX_TEST_DURATION);
        final var kinematics = trajectory.getKinematics();
        final var finalTimestamp = trajectory.getTimestamp(kinematics.size());
        final var finalFrame = trajectory.getFinalFrame();
        final var position = finalFrame.getECEFPosition();
        final var velocity = finalFrame.getECEFVelocity();

        final var elapsed = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                GATE_EPOCH_INTERVAL, createInitConfig(), new ECEFFrame(trajectory.getInitialFrame()));
        assertTrue(elapsed.update(kinematics.getFirst(), trajectory.getTimestamp(0)));
        for (var i = 0; i < kinematics.size(); i++) {
            assertTrue(elapsed.update(kinematics.get(i), trajectory.getTimestamp(i + 1)));
        }
        assertNull(elapsed.getLastStateTimestamp());
        assertTrue(elapsed.updatePositionAndVelocity(position, velocity, finalTimestamp));
        assertEquals(finalTimestamp, elapsed.getLastStateTimestamp(), 0.0);

        final var immediate = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                GATE_EPOCH_INTERVAL, createInitConfig(), new ECEFFrame(finalFrame));
        assertTrue(immediate.updatePositionAndVelocity(position, velocity, finalTimestamp));

        final var elapsedCovariance = elapsed.getState().getCovariance();
        final var immediateCovariance = immediate.getState().getCovariance();
        // both covariances would be equal if the first epoch used a zero propagation interval
        assertNotEquals(immediateCovariance, elapsedCovariance);

        // gyroscope biases are not observable in such a short time, hence their uncertainty can only
        // grow with the elapsed time because of their random walk
        for (var i = 12; i < 15; i++) {
            assertTrue(elapsedCovariance.getElementAt(i, i) > immediateCovariance.getElementAt(i, i));
        }
    }

    /**
     * Checks that with a zero epoch interval only fixes that do not advance time are ignored.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testEpochGateWithZeroEpochInterval() throws LockedException, NotReadyException, INSException {
        final var frame = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, FIX_TEST_DURATION)
                .getInitialFrame();
        final var position = frame.getECEFPosition();
        final var velocity = frame.getECEFVelocity();

        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                frame);
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);

        assertTrue(estimator.updatePositionAndVelocity(position, velocity, GATE_EPOCH_INTERVAL));
        assertFalse(estimator.updatePositionAndVelocity(position, velocity, GATE_EPOCH_INTERVAL));
        assertFalse(estimator.updatePositionAndVelocity(position, velocity, 0.0));
        assertTrue(estimator.updatePositionAndVelocity(position, velocity, GATE_THIRD_FIX_TIMESTAMP));
    }

    /**
     * Checks the closed loop on a trajectory with speed and heading changes, tactical grade IMU errors
     * and a noisy fix every second: after convergence the position error stays below three standard
     * deviations of the fix noise, it ends up smaller than the error of the INS-only solution for the
     * same data, and it is consistent with the estimated covariance.
     *
     * @throws LockedException            if estimator is locked.
     * @throws NotReadyException          if estimator is not ready.
     * @throws INSException               if estimation fails.
     * @throws InertialNavigatorException if the INS-only navigation fails.
     * @throws WrongSizeException         never thrown.
     */
    @Test
    void testClosedLoopTracksTruth() throws LockedException, NotReadyException, INSException,
            InertialNavigatorException, WrongSizeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.turningAndAccelerating(CLOSED_LOOP_DURATION);
        final var frames = trajectory.getFrames();

        final var random = new Random(SEED);
        final var measuredKinematics = new ArrayList<BodyKinematics>();
        BodyKinematicsGenerator.generate(trajectory.getSampleInterval(), trajectory.getKinematics(),
                createImuErrors(), random, measuredKinematics);
        final var positionNoise = new GaussianRandomizer(random, 0.0, FIX_POSITION_SD);
        final var velocityNoise = new GaussianRandomizer(random, 0.0, FIX_VELOCITY_SD);

        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                trajectory.getInitialFrame());

        var numEpochs = 0;
        var numConsistent = 0;
        var maxConvergedError = 0.0;
        assertTrue(estimator.update(measuredKinematics.getFirst(), trajectory.getTimestamp(0)));
        for (var i = 1; i < frames.size(); i++) {
            assertTrue(estimator.update(measuredKinematics.get(i - 1), trajectory.getTimestamp(i)));

            if (i % SAMPLES_PER_FIX == 0) {
                // a noisy fix every second
                final var truth = frames.get(i);
                assertTrue(estimator.updatePositionAndVelocity(
                        truth.getX() + positionNoise.nextDouble(), truth.getY() + positionNoise.nextDouble(),
                        truth.getZ() + positionNoise.nextDouble(), truth.getVx() + velocityNoise.nextDouble(),
                        truth.getVy() + velocityNoise.nextDouble(), truth.getVz() + velocityNoise.nextDouble(),
                        trajectory.getTimestamp(i)));

                final var state = estimator.getState();
                numEpochs++;
                if (isPositionWithinCovariance(truth, state)) {
                    numConsistent++;
                }
                if (trajectory.getTimestamp(i) >= CONVERGENCE_TIME) {
                    maxConvergedError = Math.max(maxConvergedError, positionError(truth, state.getFrame()));
                }
            }
        }

        // after convergence the position error stays below three standard deviations of the fix
        assertEquals(Math.round(CLOSED_LOOP_DURATION), numEpochs);
        assertTrue(maxConvergedError < SIGMA_FACTOR * FIX_POSITION_SD);

        // the filtered solution ends up closer to the truth than the INS-only one
        final var truth = trajectory.getFinalFrame();
        final var insOnly = KalmanFilteredEstimatorTrajectory.chainNavigator(trajectory.getInitialFrame(),
                measuredKinematics, trajectory.getSampleInterval());
        assertTrue(positionError(truth, estimator.getFrame()) < positionError(truth, insOnly));

        // position errors are consistent with the estimated covariance
        assertTrue(numConsistent >= MIN_CONSISTENT_RATIO * numEpochs);
    }

    /**
     * Checks that the closed loop corrects an initial attitude error: with error-free kinematics, an
     * INS whose yaw starts 2 degrees off and a noisy fix every second on a trajectory with speed and
     * heading changes, the yaw error decreases, the corrected attitude remains a valid rotation and
     * the position error ends up below three standard deviations of the fix noise.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSException                                  if estimation fails.
     */
    @Test
    void testClosedLoopCorrectsInitialAttitudeError() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, INSException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.turningAndAccelerating(CLOSED_LOOP_DURATION);
        final var frames = trajectory.getFrames();
        final var kinematics = trajectory.getKinematics();

        // INS starts at the true position and velocity, with a yaw error
        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(trajectory.getInitialFrame());
        final var trueC = nedFrame.getCoordinateTransformation();
        nedFrame.setCoordinateTransformation(new CoordinateTransformation(trueC.getRollEulerAngle(),
                trueC.getPitchEulerAngle(), trueC.getYawEulerAngle() + INITIAL_YAW_ERROR, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME));
        final var initialFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        assertEquals(INITIAL_YAW_ERROR, yawError(trajectory.getInitialFrame(), initialFrame), YAW_TOLERANCE);

        final var random = new Random(SEED);
        final var positionNoise = new GaussianRandomizer(random, 0.0, FIX_POSITION_SD);
        final var velocityNoise = new GaussianRandomizer(random, 0.0, FIX_VELOCITY_SD);

        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                initialFrame);

        assertTrue(estimator.update(kinematics.getFirst(), trajectory.getTimestamp(0)));
        for (var i = 1; i < frames.size(); i++) {
            assertTrue(estimator.update(kinematics.get(i - 1), trajectory.getTimestamp(i)));

            if (i % SAMPLES_PER_FIX == 0) {
                final var truth = frames.get(i);
                assertTrue(estimator.updatePositionAndVelocity(
                        truth.getX() + positionNoise.nextDouble(), truth.getY() + positionNoise.nextDouble(),
                        truth.getZ() + positionNoise.nextDouble(), truth.getVx() + velocityNoise.nextDouble(),
                        truth.getVy() + velocityNoise.nextDouble(), truth.getVz() + velocityNoise.nextDouble(),
                        trajectory.getTimestamp(i)));
            }
        }

        final var truth = trajectory.getFinalFrame();
        final var result = estimator.getFrame();
        assertTrue(Math.abs(yawError(truth, result)) < HALF * INITIAL_YAW_ERROR);
        assertTrue(positionError(truth, result) < SIGMA_FACTOR * FIX_POSITION_SD);
    }

    /**
     * Checks that a position and velocity fix cannot be provided when the estimator is not ready,
     * either because there is no frame or because there is no Kalman filter configuration.
     */
    @Test
    void testUpdatePositionAndVelocityWhenNotReady() {
        final var position = new ECEFPosition();
        final var velocity = new ECEFVelocity();

        final var estimator1 = new INSLooselyCoupledKalmanFilteredEstimator();
        assertFalse(estimator1.isPropagateReady());
        assertThrows(NotReadyException.class, () -> estimator1.updatePositionAndVelocity(position, velocity, 0.0));
        assertThrows(NotReadyException.class, () -> estimator1.updatePositionAndVelocity(position, velocity,
                new Time(0.0, TimeUnit.SECOND)));
        assertThrows(NotReadyException.class, () -> estimator1.updatePositionAndVelocity(0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0));

        // a frame without Kalman filter configuration is enough for IMU samples, but not for fixes
        final var estimator2 = new INSLooselyCoupledKalmanFilteredEstimator(new ECEFFrame());
        assertTrue(estimator2.isUpdateReady());
        assertFalse(estimator2.isPropagateReady());
        assertThrows(NotReadyException.class, () -> estimator2.updatePositionAndVelocity(position, velocity, 0.0));
    }

    /**
     * Checks that timestamps not advancing time are ignored by update without any side effect, and
     * that a null sample only moves the time reference.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testUpdateIgnoresSamplesNotAdvancingTime() throws LockedException, NotReadyException, INSException {
        final var frame = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, FIX_TEST_DURATION)
                .getInitialFrame();
        final var initialFrame = new ECEFFrame(frame);
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(frame, this);
        reset();

        assertNull(estimator.getLastKinematicsTimestamp());
        assertNull(estimator.getLastKinematicsTimestampAsTime());
        final var time = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastKinematicsTimestampAsTime(time));

        // a null sample is accepted, initialises the state and sets the time reference only
        assertTrue(estimator.update(null, FIX_TEST_DURATION));
        assertNull(estimator.getKinematics());
        assertNull(estimator.getCorrectedKinematics());
        assertNotNull(estimator.getState());
        assertEquals(FIX_TEST_DURATION, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(new Time(FIX_TEST_DURATION, TimeUnit.SECOND), estimator.getLastKinematicsTimestampAsTime());
        assertTrue(estimator.getLastKinematicsTimestampAsTime(time));
        assertEquals(new Time(FIX_TEST_DURATION, TimeUnit.SECOND), time);

        // samples not advancing time are ignored without side effects
        final var kinematics = new BodyKinematics();
        assertFalse(estimator.update(kinematics, FIX_TEST_DURATION));
        assertFalse(estimator.update(kinematics, 0.0));
        assertNull(estimator.getKinematics());
        assertEquals(FIX_TEST_DURATION, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(initialFrame, estimator.getFrame());
        assertEquals(1, updateStart);
        assertEquals(1, updateEnd);

        // a null sample advancing time performs no navigation
        assertTrue(estimator.update(null, 2.0 * FIX_TEST_DURATION));
        assertEquals(initialFrame, estimator.getFrame());
        assertEquals(2.0 * FIX_TEST_DURATION, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(2, updateStart);
        assertEquals(2, updateEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);
    }

    /**
     * Checks that setting a new frame once the Kalman filter state exists keeps the state synchronised
     * with the frame while preserving its covariance.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    @Test
    void testSetFrameSynchronizesState() throws LockedException, NotReadyException, INSException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.northAtConstantSpeed(NORTH_SPEED,
                FIX_TEST_DURATION);
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                trajectory.getInitialFrame());
        assertTrue(estimator.update(new BodyKinematics(), 0.0));

        final var state1 = estimator.getState();
        assertFrameEquals(trajectory.getInitialFrame(), state1.getFrame());

        final var newFrame = trajectory.getFinalFrame();
        estimator.setFrame(newFrame);

        final var state2 = estimator.getState();
        assertFrameEquals(newFrame, state2.getFrame());
        assertEquals(state1.getCovariance(), state2.getCovariance());
    }

    @Override
    public void onUpdateStart(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateStart++;
    }

    @Override
    public void onUpdateEnd(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateEnd++;
    }

    @Override
    public void onPropagateStart(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateStart++;
    }

    @Override
    public void onPropagateEnd(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateEnd++;
    }

    @Override
    public void onReset(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        reset++;
    }

    private void reset() {
        updateStart = 0;
        updateEnd = 0;
        propagateStart = 0;
        propagateEnd = 0;
        reset = 0;
    }

    @SuppressWarnings("deprecation")
    private static void checkLocked(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setConfig(null));
        assertThrows(LockedException.class, () -> estimator.setFrame(null));
        assertThrows(LockedException.class, () -> estimator.setInitialConfig(null));
        assertThrows(LockedException.class, () -> estimator.update(null, 0.0));
        assertThrows(LockedException.class, () -> estimator.update(null,
                new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.propagate(0.0));
        assertThrows(LockedException.class, () -> estimator.propagate(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.updatePositionAndVelocity(new ECEFPosition(),
                new ECEFVelocity(), 0.0));
        assertThrows(LockedException.class, () -> estimator.updatePositionAndVelocity(new ECEFPosition(),
                new ECEFVelocity(), new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.updatePositionAndVelocity(0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, estimator::reset);
    }

    private static INSLooselyCoupledKalmanInitializerConfig generateInitConfig() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSLooselyCoupledKalmanInitializerConfig(initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
    }

    private static INSLooselyCoupledKalmanConfig generateKalmanConfig() {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD,
                positionNoiseSD, velocityNoiseSD);
    }

    /**
     * Feeds a whole trajectory to an estimator: the first kinematics sample is provided at the start
     * time (which only initialises the estimator), then one sample at the end of each sample interval.
     *
     * @param estimator  estimator to be fed.
     * @param trajectory trajectory providing the error-free kinematics.
     * @return number of samples accepted by the estimator.
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSException      if estimation fails.
     */
    private static int feed(
            final INSLooselyCoupledKalmanFilteredEstimator estimator,
            final KalmanFilteredEstimatorTrajectory trajectory) throws LockedException, NotReadyException,
            INSException {
        final var kinematics = trajectory.getKinematics();
        var accepted = estimator.update(kinematics.getFirst(), trajectory.getTimestamp(0)) ? 1 : 0;
        for (var i = 0; i < kinematics.size(); i++) {
            if (estimator.update(kinematics.get(i), trajectory.getTimestamp(i + 1))) {
                accepted++;
            }
        }
        return accepted;
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
     * Difference between the yaw angles of two frames.
     *
     * @param expected expected frame.
     * @param actual   actual frame.
     * @return yaw difference expressed in radians.
     */
    private static double yawError(final ECEFFrame expected, final ECEFFrame actual) {
        final var expectedYaw = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(expected)
                .getCoordinateTransformation().getYawEulerAngle();
        final var actualYaw = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(actual)
                .getCoordinateTransformation().getYawEulerAngle();
        return actualYaw - expectedYaw;
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
     * Creates the deterministic Kalman filter configuration used by the trajectory tests (tactical
     * grade values of Groves' {@code INS_GNSS_Demo_3}).
     *
     * @return Kalman filter configuration.
     */
    private static INSLooselyCoupledKalmanConfig createKalmanConfig() {
        return new INSLooselyCoupledKalmanConfig(GYRO_NOISE_PSD, ACCELEROMETER_NOISE_PSD, ACCELEROMETER_BIAS_PSD,
                GYRO_BIAS_PSD, FIX_POSITION_SD, FIX_VELOCITY_SD);
    }

    /**
     * Creates the deterministic initial uncertainties used by the trajectory tests (values of Groves'
     * {@code INS_GNSS_Demo_3}).
     *
     * @return Kalman filter initializer configuration.
     */
    private static INSLooselyCoupledKalmanInitializerConfig createInitConfig() {
        return new INSLooselyCoupledKalmanInitializerConfig(INITIAL_ATTITUDE_UNCERTAINTY,
                INITIAL_VELOCITY_UNCERTAINTY, INITIAL_POSITION_UNCERTAINTY, INITIAL_ACCELEROMETER_BIAS_UNCERTAINTY,
                INITIAL_GYRO_BIAS_UNCERTAINTY);
    }

    /**
     * Creates a frame with a seeded pseudo-random position, velocity and attitude.
     *
     * @param randomizer seeded randomizer.
     * @return a new ECEF frame.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    private static ECEFFrame createFrame(final UniformRandomizer randomizer)
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

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        return new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);
    }

    /**
     * Creates a frame equal to the provided one but located {@link #NORTH_OFFSET} meters north of it.
     *
     * @param frame reference frame.
     * @return a new ECEF frame north of the reference.
     */
    private static ECEFFrame createFrameNorthOf(final ECEFFrame frame) {
        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(frame);
        final var radii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(nedFrame.getLatitude());
        nedFrame.setLatitude(nedFrame.getLatitude() + NORTH_OFFSET / (radii.getRn() + nedFrame.getHeight()));
        return NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
    }

    /**
     * Creates the errors of a tactical grade IMU (values of Groves' {@code INS_GNSS_Demo_3}).
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
     * Checks the behaviour of update for the first two samples: the first one only initialises the
     * estimator (state created from the frame, no navigation, no Kalman filter epoch), a repeated or
     * older timestamp is ignored, and the second one is integrated with the ECEF inertial navigation
     * equations over the elapsed interval.
     *
     * @param frame       initial frame. It is updated in place by the estimator.
     * @param kinematics  kinematics provided on both samples.
     * @param timeSeconds timestamp of the first sample, and interval up to the second one, expressed
     *                    in seconds.
     * @return frame estimated after the second sample.
     * @throws LockedException            if estimator is locked.
     * @throws NotReadyException          if estimator is not ready.
     * @throws INSException               if estimation fails.
     * @throws InertialNavigatorException if the oracle navigation fails.
     */
    private ECEFFrame assertUpdateIntegratesSample(
            final ECEFFrame frame, final BodyKinematics kinematics, final double timeSeconds)
            throws LockedException, NotReadyException, INSException, InertialNavigatorException {
        final var initialFrame = new ECEFFrame(frame);
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), createInitConfig(),
                frame, this);

        reset();
        assertEquals(0, updateStart);
        assertEquals(0, updateEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);

        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getLastKinematicsTimestamp());
        assertTrue(estimator.isUpdateReady());
        assertTrue(estimator.isPropagateReady());

        // first sample only initialises the estimator: no navigation and no Kalman filter epoch
        assertTrue(estimator.update(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastKinematicsTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastKinematicsTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
        assertNull(estimator.getLastStateTimestamp());
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(kinematics, estimator.getKinematics());
        // no bias has been estimated yet, so corrected kinematics equal provided ones
        assertEquals(kinematics, estimator.getCorrectedKinematics());

        final var state1 = estimator.getState();
        assertNotNull(state1);
        final var state2 = new INSLooselyCoupledKalmanState();
        assertTrue(estimator.getState(state2));
        assertEquals(state1, state2);
        assertEquals(initialFrame, estimator.getFrame());
        assertFrameEquals(initialFrame, state1.getFrame());

        // a sample with the same or an older timestamp makes no action
        assertFalse(estimator.update(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));
        assertFalse(estimator.update(kinematics, HALF * timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertEquals(state1, estimator.getState());

        // a sample with a new timestamp is integrated over the elapsed interval
        assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(2.0 * timeSeconds, estimator.getLastKinematicsTimestamp(), 0.0);
        assertNull(estimator.getLastStateTimestamp());
        assertEquals(kinematics, estimator.getKinematics());
        assertEquals(kinematics, estimator.getCorrectedKinematics());

        final var expected = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame, List.of(kinematics),
                timeSeconds);
        final var result = estimator.getFrame();
        assertFrameEquals(expected, result);

        // state holds the INS solution, and its covariance is untouched without a Kalman filter epoch
        final var state3 = estimator.getState();
        assertFrameEquals(result, state3.getFrame());
        assertEquals(state1.getCovariance(), state3.getCovariance());

        // Kalman filter epochs are only notified when a fix is provided
        assertEquals(2, updateStart);
        assertEquals(2, updateEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);

        return result;
    }

    /**
     * Asserts that a frame is the result of a free fall from an initial frame: displacement
     * v0 * t + g * t^2 / 2 and velocity v0 + g * t, where g is the gravity at the initial position.
     *
     * @param initialFrame      frame at the start of the fall.
     * @param result            frame at the end of the fall.
     * @param duration          duration of the fall expressed in seconds.
     * @param positionTolerance tolerance of the displacement expressed in meters.
     * @param velocityTolerance tolerance of the velocity expressed in meters per second.
     */
    private static void assertFreeFall(
            final ECEFFrame initialFrame, final ECEFFrame result, final double duration,
            final double positionTolerance, final double velocityTolerance) {
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

        assertEquals(0.0, displacementError, positionTolerance);
        assertEquals(0.0, velocityError, velocityTolerance);
    }

    /**
     * Gets the trace of the position block of the covariance of a Kalman filter state.
     *
     * @param state Kalman filter state.
     * @return sum of the variances of the three position coordinates, expressed in squared meters.
     */
    private static double positionCovarianceTrace(final INSLooselyCoupledKalmanState state) {
        final var covariance = state.getCovariance();
        var trace = 0.0;
        for (var i = POSITION_STATE_INDEX; i < POSITION_STATE_INDEX + POSITION_COMPONENTS; i++) {
            trace += covariance.getElementAt(i, i);
        }
        return trace;
    }

    /**
     * Indicates whether the position error of a Kalman filter state lies, on every axis, within
     * {@link #SIGMA_FACTOR} standard deviations of its estimated covariance.
     *
     * @param truth true frame.
     * @param state Kalman filter state.
     * @return true if the position error is consistent with the covariance, false otherwise.
     */
    private static boolean isPositionWithinCovariance(final ECEFFrame truth, final INSLooselyCoupledKalmanState state) {
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
}
