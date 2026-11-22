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
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.navigation.gnss.GNSSConfig;
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.navigation.gnss.GNSSLeastSquaresPositionAndVelocityEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.navigation.gnss.GNSSMeasurementsGenerator;
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
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class INSGNSSLooselyCoupledKalmanFilteredEstimatorTest implements INSGNSSLooselyCoupledKalmanFilteredEstimatorListener {

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

    private static final double MIN_SAT_VELOCITY_VALUE = -3500.0;
    private static final double MAX_SAT_VELOCITY_VALUE = 3500.0;

    private static final double MIN_DEGREES_PER_SECOND = -10.0;
    private static final double MAX_DEGREES_PER_SECOND = 10.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    /**
     * Seed of the pseudo-random generators, so that every run uses exactly the same inputs.
     */
    private static final long SEED = 36L;

    /**
     * Number of satellites in view. Ten satellites is a typical GPS constellation in open sky.
     */
    private static final int NUM_SATELLITES = 10;

    /**
     * Distance, in meters, from the user to every satellite: a typical range to a GPS satellite
     * (between 2.0e7 m at the zenith and 2.6e7 m at the horizon).
     */
    private static final double SATELLITE_RANGE = 2.2e7;

    /**
     * Minimum elevation of the satellites, in degrees. It is above the masking angle so that every
     * satellite stays in view while user and satellites move.
     */
    private static final double MIN_SATELLITE_ELEVATION_DEGREES = 20.0;

    /**
     * Maximum elevation of the satellites, in degrees.
     */
    private static final double MAX_SATELLITE_ELEVATION_DEGREES = 80.0;

    /**
     * Masking angle of the receiver, in degrees (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double MASK_ANGLE_DEGREES = 10.0;

    /**
     * Receiver clock offset at time zero, in meters (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double RECEIVER_CLOCK_OFFSET = 10000.0;

    /**
     * Receiver clock drift, in meters per second (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double RECEIVER_CLOCK_DRIFT = 100.0;

    /**
     * Code tracking error standard deviation of the noisy pseudo-ranges, in meters (Groves'
     * {@code INS_GNSS_Demo_3}).
     */
    private static final double CODE_TRACKING_SD = 1.0;

    /**
     * Range rate tracking error standard deviation of the noisy pseudo-range rates, in meters per
     * second (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double RANGE_RATE_TRACKING_SD = 0.02;

    /**
     * Tolerance, in meters, for the least squares position obtained from error-free pseudo-ranges.
     * The only error of the solution comes from the Sagnac correction, which the least squares
     * estimator evaluates with the range to its prior position: with the prior within 10 m of the
     * truth (the first fix is refined once) the range to a satellite moves by
     * 7.29e-5 rad/s * 10 m / 3e8 m/s * 2.6e7 m = 6e-5 m; hence 1e-3 m.
     */
    private static final double GNSS_POSITION_TOLERANCE = 1e-3;

    /**
     * Tolerance, in meters per second, for the least squares velocity obtained from error-free
     * pseudo-range rates. The measurement generator projects the relative velocity onto the line of
     * sight before applying the Sagnac rotation, while the least squares estimator projects it
     * afterwards: both lines of sight differ by 7.29e-5 rad/s * 2.2e7 m / 3e8 m/s = 5.3e-6 rad, which
     * for a relative velocity (including the Earth rotation term) of up to 8e3 m/s amounts to
     * 4e-2 m/s per satellite (1.2e-2 m/s were observed in the solution); hence 5e-2 m/s.
     */
    private static final double GNSS_VELOCITY_TOLERANCE = 5e-2;

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
     * Epoch interval, expressed in seconds, of the tests providing GNSS measurements every second.
     */
    private static final double ONE_SECOND_EPOCH_INTERVAL = 1.0;

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
     * Tolerance, in meters per second, for the velocity reached against the analytic truth of a
     * trajectory: the navigator velocity update is the exact inverse of the kinematics estimation, so
     * the error is the one of the initial velocity, which comes from an error-free GNSS fix (see
     * {@link #GNSS_VELOCITY_TOLERANCE}).
     */
    private static final double TRUTH_VELOCITY_TOLERANCE = GNSS_VELOCITY_TOLERANCE;

    /**
     * Tolerance, in meters, for the position reached against the analytic truth of a trajectory.
     * The navigator integrates position with the trapezoidal rule (eq. 5.38), whose error is well
     * below a micrometer, so the error is the one of the initial velocity (see
     * {@link #TRUTH_VELOCITY_TOLERANCE}) accumulated over the 10 s of the trajectory: 0.5 m, which is
     * 200 times smaller than the 100 m travelled.
     */
    private static final double TRUTH_POSITION_TOLERANCE = TRUTH_VELOCITY_TOLERANCE * TRAJECTORY_DURATION;

    /**
     * Minimum distance, in meters, between the per-sample integration and a single integration step
     * over the whole epoch using only the last sample: the body ends 0.5 m ahead, while a single step
     * with the final deceleration of 2 m/s^2 would move it 1 m backwards.
     */
    private static final double SINGLE_STEP_MIN_DIFFERENCE = 1.0;

    /**
     * Tolerance, in meters, when comparing against the chained navigator oracle. Both sides execute
     * the same floating point operations, so differences can only come from rounding of ECEF
     * coordinates of magnitude 6.4e6 m (ulp of 1e-9 m).
     */
    private static final double CHAIN_POSITION_TOLERANCE = 1e-6;

    /**
     * Tolerance, in meters per second, when comparing velocity against the chained navigator oracle:
     * both sides execute the same operations, so only rounding errors (below 1e-9 m/s) remain.
     */
    private static final double CHAIN_VELOCITY_TOLERANCE = 1e-6;

    /**
     * Tolerance for each element of the body to ECEF rotation matrix when comparing against the
     * chained navigator oracle: elements are bounded by one and both sides execute the same
     * operations, so differences are bounded by accumulated rounding (1e-13 after 1000 samples).
     */
    private static final double CHAIN_ROTATION_TOLERANCE = 1e-9;

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
     * Standard deviation per axis assumed by the Kalman filter for the position of the GNSS fixes,
     * expressed in meters (Groves' {@code INS_GNSS_Demo_3}).
     */
    private static final double FIX_POSITION_SD = 2.5;

    /**
     * Standard deviation per axis assumed by the Kalman filter for the velocity of the GNSS fixes,
     * expressed in meters per second (Groves' {@code INS_GNSS_Demo_3}).
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
     * Speed, in meters per second, of the body used to check that a GNSS fix is not navigated forward
     * in time.
     */
    private static final double FAST_SPEED = 30.0;

    /**
     * Duration, in seconds, of the trajectory used to check that a GNSS fix is not navigated forward
     * in time: five GNSS epochs.
     */
    private static final double FAST_TRAJECTORY_DURATION = 5.0;

    /**
     * Maximum position error, in meters, right after a GNSS epoch with error-free measurements and an
     * error-free IMU. Using the fix as if it were valid one epoch earlier (or navigating it forward
     * over one epoch) displaces a body moving at 30 m/s by 30 m; 0.5 m is 60 times smaller, while the
     * error actually expected is the one of the least squares fix (see
     * {@link #GNSS_POSITION_TOLERANCE}).
     */
    private static final double FIX_NOT_ADVANCED_TOLERANCE = 0.5;

    /**
     * Epoch interval, in seconds, used to check the epoch gate. It is exactly representable in
     * binary floating point, as are the measurement timestamps, so that intervals are exact.
     */
    private static final double GATE_EPOCH_INTERVAL = 0.5;

    /**
     * Timestamp, in seconds, of the measurements arriving exactly two epoch intervals after the first
     * ones.
     */
    private static final double GATE_THIRD_FIX_TIMESTAMP = 1.0;

    /**
     * Timestamp, in seconds, of the measurements arriving only half an epoch interval after the
     * previous ones.
     */
    private static final double GATE_EARLY_FIX_TIMESTAMP = 1.25;

    /**
     * Duration, in seconds, of the closed-loop trajectory.
     */
    private static final double CLOSED_LOOP_DURATION = 60.0;

    /**
     * Number of IMU samples between consecutive GNSS epochs (1 s at 100 Hz).
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

    private int updateGNSSMeasurementsStart;
    private int updateGNSSMeasurementsEnd;
    private int updateBodyKinematicsStart;
    private int updateBodyKinematicsEnd;
    private int propagateStart;
    private int propagateEnd;
    private int propagateEndAtUpdateGNSSMeasurementsEnd;
    private int reset;

    @Test
    void testConstructor() throws InvalidSourceAndDestinationFrameTypeException {

        // test constructor 1
        var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 2
        final var kalmanConfig = generateKalmanConfig();
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime2);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime2);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        INSLooselyCoupledKalmanConfig kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 3
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime3 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime3);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime3);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval));

        // test constructor 4
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime4 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime4);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime4);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 5
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime5 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime5);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime5);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval));

        // test constructor 6
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime6 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime6);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime6);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 7
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime7 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime7);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime7);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, this));

        // test constructor 8
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime8 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime8);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime8);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, this));

        // test constructor 9
        final var epochIntervalTime9 = new Time(epochInterval, TimeUnit.SECOND);
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime9);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime10 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime10);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime10);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 10
        epochIntervalTime.setValue(epochInterval);
        epochIntervalTime.setUnit(TimeUnit.SECOND);
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime11 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime11);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime11);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        final var wrongEpochIntervalTime = new Time(-epochInterval, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime));

        // test constructor 11
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime12 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime12);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime12);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, this));

        // test constructor 12
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime13 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime13);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime13);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, this));

        // test constructor 13
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime14 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime14);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime14);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        CoordinateTransformation c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, 
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 14
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime15 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime15);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime15);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 15
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime16 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime16);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime16);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, c));

        // test constructor 16
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime17 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime17);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime17);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 17
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime18 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime18);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime18);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, c));

        // test constructor 18
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime19 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime19);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime19);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, 
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 19
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime20 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime20);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime20);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, c, this));

        // test constructor 20
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime21 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime21);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime21);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, 
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, c, this));

        // test constructor 21
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime22 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime22);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime22);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, c));

        // test constructor 22
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime23 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime23);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime23);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, c));

        // test constructor 23
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime24 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime24);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime24);
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, c, this));

        // test constructor 24
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        final var epochIntervalTime25 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime25);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime25);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, 
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, c, this));

        // test constructor 25
        final var initialConfig = generateInitConfig();
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime26 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime26);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime26);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        INSLooselyCoupledKalmanInitializerConfig initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 26
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime27 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime27);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime27);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 27
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime28 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime28);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime28);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig));

        // test constructor 28
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime29 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime29);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime29);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 29
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime30 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime30);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime30);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig));

        // test constructor 30
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime31 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime31);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime31);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 31
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime32 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime32);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime32);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, this));

        // test constructor 32
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime33 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime33);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime33);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, this));

        // test constructor 33
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime34 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime34);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime34);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig));

        // test constructor 34
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        final var epochIntervalTime35 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime35);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime35);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig));

        // test constructor 35
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        final var epochIntervalTime36 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime36);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime36);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, this));

        // test constructor 36
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        final var epochIntervalTime37 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime37);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime37);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, this));

        // test constructor 37
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime38 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime38);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime38);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 38
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime39 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime39);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime39);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 39
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime40 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime40);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime40);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, c));

        // test constructor 40
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime41 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime41);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime41);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 41
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime42 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime42);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime42);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, c));

        // test constructor 42
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime43 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime43);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime43);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 43
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime44 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime44);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime44);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, c, this));

        // test constructor 44
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, c,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime45 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime45);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime45);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, c, this));

        // test constructor 45
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime46 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime46);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime46);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, c));

        // test constructor 46
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime47 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime47);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime47);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, c));

        // test constructor 47
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime48 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime48);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime48);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, c, this));

        // test constructor 48
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, c,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime49 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime49);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime49);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
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
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, c, this));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetEpochInterval() throws LockedException {
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator();

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
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator();

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
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator();

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
    void testGetSetCoordinateTransformation() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator();

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
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setCoordinateTransformation(new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)));
    }

    @Test
    void testGetSetInitialConfig() throws LockedException {
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator();

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

    @Test
    void testIsUpdateMeasurementsReady() {
        assertFalse(INSGNSSLooselyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(null));

        final var measurements = new ArrayList<GNSSMeasurement>();
        assertFalse(INSGNSSLooselyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));

        for (var i = 0; i < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS; i++) {
            measurements.add(new GNSSMeasurement());
        }
        assertTrue(INSGNSSLooselyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));
    }

    /**
     * Checks the first GNSS measurements provided to an estimator without initial attitude and without
     * kinematics: the least squares fix initialises the INS solution (keeping the default attitude), a
     * Kalman filter epoch is run with the fix as measurement, and measurements repeating the timestamp
     * are ignored.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSGNSSException  if estimation fails.
     */
    @Test
    void testUpdateMeasurementsWithoutKinematicsAndWithoutInitialAttitude() throws LockedException, NotReadyException,
            INSGNSSException {
        final var fixture = createFixture();
        final var timeSeconds = fixture.timeSeconds();
        final var measurements = fixture.measurements();

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        reset();
        assertNotInitialized(estimator);
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));

        // update measurement
        assertTrue(estimator.updateMeasurements(measurements, timeSeconds));

        final var defaultC = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertFirstFix(estimator, fixture, timeSeconds, defaultC);
        assertNull(estimator.getKinematics());
        assertNull(estimator.getCorrectedKinematics());

        // update again with the same timestamp makes no action
        final var estimation1 = estimator.getEstimation();
        final var state1 = estimator.getState();
        assertFalse(estimator.updateMeasurements(measurements, new Time(timeSeconds, TimeUnit.SECOND)));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(estimation1, estimator.getEstimation());
        assertEquals(state1, estimator.getState());

        // GNSS measurements run a Kalman filter epoch, but they are not navigated as an IMU sample
        assertEquals(1, updateGNSSMeasurementsStart);
        assertEquals(1, updateGNSSMeasurementsEnd);
        assertEquals(0, updateBodyKinematicsStart);
        assertEquals(0, updateBodyKinematicsEnd);
        assertEquals(1, propagateStart);
        assertEquals(1, propagateEnd);
    }

    /**
     * Checks the first GNSS measurements provided to an estimator without initial attitude after an
     * IMU sample: the sample provided before any fix does not prevent the fix from initialising the
     * INS solution, and a Kalman filter epoch is run with the fix as measurement.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSGNSSException  if estimation fails.
     */
    @Test
    void testUpdateMeasurementsWithBodyKinematicsAndWithoutInitialAttitude() throws LockedException, INSGNSSException,
            NotReadyException {
        final var fixture = createFixture();
        final var timeSeconds = fixture.timeSeconds();

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        final var defaultC = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var kinematics = new BodyKinematics();
        assertKinematicsBeforeFirstFix(estimator, kinematics, timeSeconds, defaultC);

        reset();

        // update measurement
        assertTrue(estimator.updateMeasurements(fixture.measurements(), 2.0 * timeSeconds));

        assertFirstFix(estimator, fixture, 2.0 * timeSeconds, defaultC);
        assertEquals(kinematics, estimator.getKinematics());
        assertEquals(kinematics, estimator.getCorrectedKinematics());

        assertEquals(1, updateGNSSMeasurementsStart);
        assertEquals(1, updateGNSSMeasurementsEnd);
        assertEquals(0, updateBodyKinematicsStart);
        assertEquals(0, updateBodyKinematicsEnd);
        assertEquals(1, propagateStart);
        assertEquals(1, propagateEnd);
    }

    /**
     * Checks the first GNSS measurements provided to an estimator with an initial attitude and without
     * kinematics: the least squares fix initialises the position and velocity of the INS solution
     * while the configured attitude is preserved.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     */
    @Test
    void testUpdateMeasurementsWithoutBodyKinematicsAndWithInitialAttitude()
            throws InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            INSGNSSException {
        final var fixture = createFixture();
        final var timeSeconds = fixture.timeSeconds();
        final var c = fixture.userFrame().getCoordinateTransformation();

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), c, this);

        reset();
        assertNotInitialized(estimator);
        assertEquals(c, estimator.getCoordinateTransformation());
        final var c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);

        // update measurement
        assertTrue(estimator.updateMeasurements(fixture.measurements(), timeSeconds));

        assertFirstFix(estimator, fixture, timeSeconds, c);
        assertNull(estimator.getKinematics());
        assertNull(estimator.getCorrectedKinematics());

        assertEquals(1, updateGNSSMeasurementsStart);
        assertEquals(1, updateGNSSMeasurementsEnd);
        assertEquals(0, updateBodyKinematicsStart);
        assertEquals(0, updateBodyKinematicsEnd);
        assertEquals(1, propagateStart);
        assertEquals(1, propagateEnd);
    }

    /**
     * Checks the first GNSS measurements provided to an estimator with an initial attitude after an
     * IMU sample: the least squares fix initialises the position and velocity of the INS solution
     * while the configured attitude is preserved.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws NotReadyException                             if estimator is not ready.
     */
    @Test
    void testUpdateMeasurementsWithKinematicsAndInitialAttitude() throws LockedException, INSGNSSException,
            InvalidSourceAndDestinationFrameTypeException, NotReadyException {
        final var fixture = createFixture();
        final var timeSeconds = fixture.timeSeconds();
        final var c = fixture.userFrame().getCoordinateTransformation();

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), c, this);

        final var kinematics = new BodyKinematics();
        assertKinematicsBeforeFirstFix(estimator, kinematics, timeSeconds, c);

        reset();

        // update measurement
        assertTrue(estimator.updateMeasurements(fixture.measurements(), 2.0 * timeSeconds));

        assertFirstFix(estimator, fixture, 2.0 * timeSeconds, c);
        assertEquals(kinematics, estimator.getKinematics());
        assertEquals(kinematics, estimator.getCorrectedKinematics());

        assertEquals(1, updateGNSSMeasurementsStart);
        assertEquals(1, updateGNSSMeasurementsEnd);
        assertEquals(0, updateBodyKinematicsStart);
        assertEquals(0, updateBodyKinematicsEnd);
        assertEquals(1, propagateStart);
        assertEquals(1, propagateEnd);
    }

    @Test
    void testUpdateMeasurementsWhenNotReadyThrowsNotReadyException() {
        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig);

        assertThrows(NotReadyException.class, () -> estimator.updateMeasurements(Collections.emptyList(),
                0.0));
    }

    /**
     * Checks that without a Kalman filter configuration GNSS measurements only initialise the position
     * and velocity of the INS solution: no Kalman filter epoch is run, and IMU samples are navigated
     * from the least squares fix.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSGNSSException  if estimation fails.
     */
    @Test
    void testUpdateMeasurementsWithoutKalmanConfig() throws LockedException, NotReadyException, INSGNSSException {
        final var fixture = createFixture();
        final var timeSeconds = fixture.timeSeconds();

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(this);

        reset();
        assertNotInitialized(estimator);
        assertNull(estimator.getConfig());

        // update measurement
        assertTrue(estimator.updateMeasurements(fixture.measurements(), timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(fixture.measurements(), estimator.getMeasurements());
        final var estimation = estimator.getEstimation();
        assertFixEqualsTruth(fixture.userFrame(), estimation);

        // no Kalman filter epoch has been run
        assertNull(estimator.getState());
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.isPropagateReady());
        assertFalse(estimator.propagate(timeSeconds));

        // the INS solution has been initialised with the fix
        assertTrue(estimator.updateBodyKinematics(new BodyKinematics(), timeSeconds));

        assertStateEqualsFix(estimation, estimator.getState());
        assertNull(estimator.getLastStateTimestamp());

        assertEquals(1, updateGNSSMeasurementsStart);
        assertEquals(1, updateGNSSMeasurementsEnd);
        assertEquals(1, updateBodyKinematicsStart);
        assertEquals(1, updateBodyKinematicsEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);
    }

    /**
     * Checks that after a GNSS fix an IMU sample with zero specific force and zero angular rate is
     * integrated as a free fall, and that the first sample only initialises the time reference.
     *
     * @throws LockedException            if estimator is locked.
     * @throws NotReadyException          if estimator is not ready.
     * @throws INSGNSSException           if estimation fails.
     * @throws InertialNavigatorException if the oracle navigation fails.
     */
    @Test
    void testUpdateKinematicsWithZeroSpecificForceAndAngularRate() throws LockedException, NotReadyException,
            INSGNSSException, InertialNavigatorException {
        final var fixture = createFixture();

        final var kinematics = new BodyKinematics();
        final var sample = assertUpdateBodyKinematicsIntegratesSample(fixture, kinematics);

        assertFreeFall(sample.initialFrame(), sample.finalFrame(), fixture.timeSeconds());
    }

    /**
     * Checks that after a GNSS fix an IMU sample with specific force and zero angular rate is
     * integrated with the ECEF inertial navigation equations, and that the first sample only
     * initialises the time reference.
     *
     * @throws LockedException            if estimator is locked.
     * @throws NotReadyException          if estimator is not ready.
     * @throws INSGNSSException           if estimation fails.
     * @throws InertialNavigatorException if the oracle navigation fails.
     */
    @Test
    void testUpdateKinematicsWithFreeFallSpecificForceAndZeroAngularRate() throws LockedException, NotReadyException,
            INSGNSSException, InertialNavigatorException {
        final var fixture = createFixture();
        final var userFrame = fixture.userFrame();

        // because there is no attitude, the specific force is directly the gravity.
        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(userFrame.getX(), userFrame.getY(),
                userFrame.getZ());
        final var kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                0.0, 0.0, 0.0);
        final var sample = assertUpdateBodyKinematicsIntegratesSample(fixture, kinematics);

        // provided specific force makes the body move differently from a free fall
        final var freeFall = KalmanFilteredEstimatorTrajectory.chainNavigator(sample.initialFrame(),
                List.of(new BodyKinematics()), fixture.timeSeconds());
        assertTrue(velocityError(freeFall, sample.finalFrame()) > FREE_FALL_VELOCITY_TOLERANCE);
    }

    /**
     * Checks that after a GNSS fix an IMU sample with zero specific force and angular rate is
     * integrated as a free fall while the attitude is rotated, and that the first sample only
     * initialises the time reference.
     *
     * @throws LockedException            if estimator is locked.
     * @throws INSGNSSException           if estimation fails.
     * @throws NotReadyException          if estimator is not ready.
     * @throws InertialNavigatorException if the oracle navigation fails.
     */
    @Test
    void testUpdateKinematicsWithZeroSpecificForceAndRotationOnly() throws LockedException, INSGNSSException,
            NotReadyException, InertialNavigatorException {
        final var fixture = createFixture();

        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var kinematics = new BodyKinematics(0.0, 0.0, 0.0, angularRateX, angularRateY, angularRateZ);
        final var sample = assertUpdateBodyKinematicsIntegratesSample(fixture, kinematics);

        assertFreeFall(sample.initialFrame(), sample.finalFrame(), fixture.timeSeconds());

        // attitude has been rotated (the oracle comparison has already checked its value)
        assertFalse(sample.initialFrame().getCoordinateTransformationMatrix().equals(
                sample.finalFrame().getCoordinateTransformationMatrix(), CHAIN_ROTATION_TOLERANCE));
    }

    /**
     * Checks that after a GNSS fix an IMU sample with both specific force and angular rate is
     * integrated with the ECEF inertial navigation equations, and that the first sample only
     * initialises the time reference.
     *
     * @throws LockedException            if estimator is locked.
     * @throws NotReadyException          if estimator is not ready.
     * @throws INSGNSSException           if estimation fails.
     * @throws InertialNavigatorException if the oracle navigation fails.
     */
    @Test
    void testUpdateKinematicsWithFreeFallSpecificForceAndRotation() throws LockedException, NotReadyException,
            INSGNSSException, InertialNavigatorException {
        final var fixture = createFixture();
        final var userFrame = fixture.userFrame();

        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(userFrame.getX(), userFrame.getY(),
                userFrame.getZ());
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
        final var kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                angularRateX, angularRateY, angularRateZ);
        final var sample = assertUpdateBodyKinematicsIntegratesSample(fixture, kinematics);

        // provided specific force makes the body move differently from a free fall
        final var freeFall = KalmanFilteredEstimatorTrajectory.chainNavigator(sample.initialFrame(),
                List.of(new BodyKinematics()), fixture.timeSeconds());
        assertTrue(velocityError(freeFall, sample.finalFrame()) > FREE_FALL_VELOCITY_TOLERANCE);

        // attitude has been rotated (the oracle comparison has already checked its value)
        assertFalse(sample.initialFrame().getCoordinateTransformationMatrix().equals(
                sample.finalFrame().getCoordinateTransformationMatrix(), CHAIN_ROTATION_TOLERANCE));
    }

    /**
     * Checks that propagate only keeps the Kalman filter state synchronised with the INS solution: it
     * never runs a Kalman filter epoch, never notifies the listener, returns false, and does not turn
     * the INS solution into a GNSS estimation.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSGNSSException  if estimation fails.
     */
    @Test
    void testPropagate() throws LockedException, NotReadyException, INSGNSSException {
        final var fixture = createFixture();
        final var timeSeconds = fixture.timeSeconds();

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        reset();
        assertNotInitialized(estimator);
        assertFalse(estimator.isPropagateReady());

        // update measurement
        assertTrue(estimator.updateMeasurements(fixture.measurements(), timeSeconds));

        assertTrue(estimator.isPropagateReady());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        final var estimation1 = estimator.getEstimation();
        final var state1 = estimator.getState();
        assertFixEqualsTruth(fixture.userFrame(), estimation1);
        assertStateEqualsFix(estimation1, state1);

        // propagate runs no Kalman filter epoch: only GNSS measurements do
        assertFalse(estimator.propagate(2.0 * timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(estimation1, estimator.getEstimation());
        assertEquals(state1, estimator.getState());

        assertFalse(estimator.propagate(new Time(2.0 * timeSeconds, TimeUnit.SECOND)));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(estimation1, estimator.getEstimation());
        assertEquals(state1, estimator.getState());

        assertEquals(1, updateGNSSMeasurementsStart);
        assertEquals(1, updateGNSSMeasurementsEnd);
        assertEquals(0, updateBodyKinematicsStart);
        assertEquals(0, updateBodyKinematicsEnd);
        assertEquals(1, propagateStart);
        assertEquals(1, propagateEnd);
    }

    @Test
    void testPropagateWhenNotReadyReturnsFalse() throws LockedException, INSGNSSException {
        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig);

        assertFalse(estimator.propagate(0.0));
    }

    /**
     * Checks that reset discards measurements, estimation, state, timestamps and INS solution, so
     * that the same GNSS measurements provided again initialise the estimator exactly as the first
     * time.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSGNSSException  if estimation fails.
     */
    @Test
    void testReset() throws LockedException, NotReadyException, INSGNSSException {
        final var fixture = createFixture();
        final var timeSeconds = fixture.timeSeconds();
        final var measurements = fixture.measurements();

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        reset();
        assertNotInitialized(estimator);

        // update measurement
        assertTrue(estimator.updateMeasurements(measurements, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        final var estimation1 = estimator.getEstimation();
        final var state1 = estimator.getState();
        assertFixEqualsTruth(fixture.userFrame(), estimation1);
        assertStateEqualsFix(estimation1, state1);

        // reset
        assertEquals(0, reset);

        estimator.reset();

        assertEquals(1, reset);
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertNull(estimator.getState());
        assertNull(estimator.getLastStateTimestamp());
        assertNull(estimator.getCoordinateTransformation());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.isRunning());

        // update again with the same timestamp now it does make an action
        assertTrue(estimator.updateMeasurements(measurements, new Time(timeSeconds, TimeUnit.SECOND)));

        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(estimation1, estimator.getEstimation());
        assertEquals(state1, estimator.getState());

        assertEquals(2, updateGNSSMeasurementsStart);
        assertEquals(2, updateGNSSMeasurementsEnd);
        assertEquals(0, updateBodyKinematicsStart);
        assertEquals(0, updateBodyKinematicsEnd);
        assertEquals(2, propagateStart);
        assertEquals(2, propagateEnd);
    }

    /**
     * Checks that the angular rate of every IMU sample is integrated: after a 10 s trajectory yawing at
     * 0.1 rad/s, initialised with GNSS measurements at the start, the estimated yaw must be 1 rad.
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
        final var estimator = createInitializedEstimator(trajectory, 0.0);
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);

        feed(estimator, trajectory);

        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(estimator.getState().getFrame());
        assertEquals(EXPECTED_FINAL_YAW, nedFrame.getCoordinateTransformation().getYawEulerAngle(), YAW_TOLERANCE);
    }

    /**
     * Checks that the specific force of every IMU sample is integrated: after moving north at 10 m/s
     * for 10 s, initialised with GNSS measurements at the start, the estimated solution must be the
     * per-sample navigation of the initial solution, and its position and velocity must be the true
     * ones up to the error of the GNSS initialisation.
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
        final var estimator = createInitializedEstimator(trajectory, 0.0);
        final var initialFrame = estimator.getState().getFrame();

        feed(estimator, trajectory);

        final var result = estimator.getState().getFrame();
        assertFrameEquals(KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame, trajectory.getKinematics(),
                trajectory.getSampleInterval()), result);

        final var expected = trajectory.getFinalFrame();
        assertEquals(0.0, positionError(expected, result), TRUTH_POSITION_TOLERANCE);
        assertEquals(0.0, velocityError(expected, result), TRUTH_VELOCITY_TOLERANCE);
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
        final var estimator = createInitializedEstimator(trajectory, ONE_SECOND_EPOCH_INTERVAL);
        final var initialFrame = estimator.getState().getFrame();

        final var integrated = feed(estimator, trajectory);

        final var kinematics = trajectory.getKinematics();
        final var expected = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame, kinematics,
                trajectory.getSampleInterval());
        final var result = estimator.getState().getFrame();
        assertFrameEquals(expected, result);

        // a single step over the whole epoch using only the last sample gives a different solution
        final var singleStep = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(ONE_SECOND_EPOCH_INTERVAL, initialFrame, kinematics.getLast(),
                singleStep);
        assertTrue(positionError(singleStep, result) > SINGLE_STEP_MIN_DIFFERENCE);

        // the initial sample plus one per interval were all accepted
        assertEquals(kinematics.size() + 1, integrated);
    }

    /**
     * Checks that a GNSS fix is used as a measurement valid at its own timestamp instead of being
     * navigated forward in time: for a body moving at 30 m/s with error-free IMU samples and error-free
     * GNSS measurements every second, the position right after each GNSS epoch must be the true one.
     *
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     */
    @Test
    void testGnssFixIsNotAdvancedInTime() throws LockedException, NotReadyException, INSGNSSException,
            InvalidSourceAndDestinationFrameTypeException {
        final var trajectory = KalmanFilteredEstimatorTrajectory.northAtConstantSpeed(FAST_SPEED,
                FAST_TRAJECTORY_DURATION);
        final var frames = trajectory.getFrames();
        final var kinematics = trajectory.getKinematics();

        final var satellites = createSatellites(new UniformRandomizer(new Random(SEED)), frames.getFirst());
        final var gnssConfig = createGnssConfig(0.0, 0.0);
        final var random = new Random(SEED);

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                ONE_SECOND_EPOCH_INTERVAL, createInitConfig(), frames.getFirst().getCoordinateTransformation());

        assertTrue(estimator.updateMeasurements(
                generateMeasurements(0.0, satellites, frames.getFirst(), gnssConfig, random), 0.0));
        assertTrue(positionError(frames.getFirst(), estimator.getState().getFrame()) < FIX_NOT_ADVANCED_TOLERANCE);

        var numEpochs = 0;
        estimator.updateBodyKinematics(kinematics.getFirst(), trajectory.getTimestamp(0));
        for (var i = 1; i < frames.size(); i++) {
            estimator.updateBodyKinematics(kinematics.get(i - 1), trajectory.getTimestamp(i));

            if (i % SAMPLES_PER_FIX == 0) {
                final var truth = frames.get(i);
                final var time = trajectory.getTimestamp(i);
                assertTrue(estimator.updateMeasurements(
                        generateMeasurements(time, satellites, truth, gnssConfig, random), time));

                assertEquals(time, estimator.getLastStateTimestamp(), 0.0);
                assertTrue(positionError(truth, estimator.getState().getFrame()) < FIX_NOT_ADVANCED_TOLERANCE);
                numEpochs++;
            }
        }

        assertEquals(Math.round(FAST_TRAJECTORY_DURATION), numEpochs);
    }

    /**
     * Checks the epoch gate of GNSS measurements: measurements arriving exactly one epoch interval
     * apart are accepted, while measurements arriving earlier, or repeating a timestamp, are ignored.
     *
     * @throws LockedException   if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws INSGNSSException  if estimation fails.
     */
    @Test
    void testEpochGateAcceptsExactInterval() throws LockedException, NotReadyException, INSGNSSException {
        final var frame = KalmanFilteredEstimatorTrajectory.yawingStationary(0.0, GATE_THIRD_FIX_TIMESTAMP)
                .getInitialFrame();
        final var satellites = createSatellites(new UniformRandomizer(new Random(SEED)), frame);
        final var gnssConfig = createGnssConfig(0.0, 0.0);
        final var random = new Random(SEED);

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                GATE_EPOCH_INTERVAL, createInitConfig(), this);
        reset();

        assertTrue(estimator.updateMeasurements(generateMeasurements(0.0, satellites, frame, gnssConfig, random),
                0.0));
        assertEquals(0.0, estimator.getLastStateTimestamp(), 0.0);

        assertTrue(estimator.updateMeasurements(
                generateMeasurements(GATE_EPOCH_INTERVAL, satellites, frame, gnssConfig, random),
                GATE_EPOCH_INTERVAL));
        assertEquals(GATE_EPOCH_INTERVAL, estimator.getLastStateTimestamp(), 0.0);

        assertTrue(estimator.updateMeasurements(
                generateMeasurements(GATE_THIRD_FIX_TIMESTAMP, satellites, frame, gnssConfig, random),
                new Time(GATE_THIRD_FIX_TIMESTAMP, TimeUnit.SECOND)));
        assertEquals(GATE_THIRD_FIX_TIMESTAMP, estimator.getLastStateTimestamp(), 0.0);

        final var estimation = estimator.getEstimation();
        final var state = estimator.getState();
        final var measurements = estimator.getMeasurements();

        // measurements arriving before a whole epoch interval has elapsed are ignored
        final var early = generateMeasurements(GATE_EARLY_FIX_TIMESTAMP, satellites, frame, gnssConfig, random);
        assertFalse(estimator.updateMeasurements(early, GATE_EARLY_FIX_TIMESTAMP));
        // measurements repeating the timestamp of the last epoch are ignored
        assertFalse(estimator.updateMeasurements(early, GATE_THIRD_FIX_TIMESTAMP));
        // measurements older than the last epoch are ignored
        assertFalse(estimator.updateMeasurements(early, GATE_EPOCH_INTERVAL));

        assertEquals(GATE_THIRD_FIX_TIMESTAMP, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(estimation, estimator.getEstimation());
        assertEquals(state, estimator.getState());
        assertEquals(measurements, estimator.getMeasurements());
        assertEquals(3, updateGNSSMeasurementsStart);
        assertEquals(3, updateGNSSMeasurementsEnd);
        assertEquals(3, propagateStart);
        assertEquals(3, propagateEnd);
        assertEquals(0, updateBodyKinematicsStart);
        assertEquals(0, updateBodyKinematicsEnd);

        // end of each GNSS measurements update is notified after its Kalman filter epoch
        assertEquals(propagateEnd, propagateEndAtUpdateGNSSMeasurementsEnd);
        assertEquals(3, propagateEndAtUpdateGNSSMeasurementsEnd);
    }

    /**
     * Checks the closed loop on a trajectory with speed and heading changes, tactical grade IMU errors
     * and noisy pseudo-ranges every second: after convergence the position error stays below three
     * standard deviations of the fix noise, it ends up smaller than the error of the INS-only solution
     * for the same data, and it is consistent with the estimated covariance.
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

        final var random = new Random(SEED);
        final var measuredKinematics = new ArrayList<BodyKinematics>();
        BodyKinematicsGenerator.generate(trajectory.getSampleInterval(), trajectory.getKinematics(),
                createImuErrors(), random, measuredKinematics);
        final var satellites = createSatellites(new UniformRandomizer(new Random(SEED)), frames.getFirst());
        final var gnssConfig = createGnssConfig(CODE_TRACKING_SD, RANGE_RATE_TRACKING_SD);

        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                ONE_SECOND_EPOCH_INTERVAL, createInitConfig(), frames.getFirst().getCoordinateTransformation());

        // noisy pseudo-ranges at the start initialise the position and velocity
        assertTrue(estimator.updateMeasurements(
                generateMeasurements(0.0, satellites, frames.getFirst(), gnssConfig, random), 0.0));

        var numEpochs = 0;
        var numConsistent = 0;
        var maxConvergedError = 0.0;
        assertTrue(estimator.updateBodyKinematics(measuredKinematics.getFirst(), trajectory.getTimestamp(0)));
        for (var i = 1; i < frames.size(); i++) {
            assertTrue(estimator.updateBodyKinematics(measuredKinematics.get(i - 1), trajectory.getTimestamp(i)));

            if (i % SAMPLES_PER_FIX == 0) {
                // noisy pseudo-ranges every second
                final var truth = frames.get(i);
                final var time = trajectory.getTimestamp(i);
                assertTrue(estimator.updateMeasurements(
                        generateMeasurements(time, satellites, truth, gnssConfig, random), time));

                final var state = estimator.getState();
                numEpochs++;
                if (isPositionWithinCovariance(truth, state)) {
                    numConsistent++;
                }
                if (time >= CONVERGENCE_TIME) {
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
        assertTrue(positionError(truth, estimator.getState().getFrame()) < positionError(truth, insOnly));

        // position errors are consistent with the estimated covariance
        assertTrue(numConsistent >= MIN_CONSISTENT_RATIO * numEpochs);
    }

    @Override
    public void onUpdateGNSSMeasurementsStart(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateGNSSMeasurementsStart++;
    }

    @Override
    public void onUpdateGNSSMeasurementsEnd(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateGNSSMeasurementsEnd++;
        propagateEndAtUpdateGNSSMeasurementsEnd = propagateEnd;
    }

    @Override
    public void onUpdateBodyKinematicsStart(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateBodyKinematicsStart++;
    }

    @Override
    public void onUpdateBodyKinematicsEnd(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateBodyKinematicsEnd++;
    }

    @Override
    public void onPropagateStart(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateStart++;
    }

    @Override
    public void onPropagateEnd(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateEnd++;
    }

    @Override
    public void onReset(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
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
        propagateEndAtUpdateGNSSMeasurementsEnd = 0;
        reset = 0;
    }

    private static void checkLocked(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
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
     * Creates the deterministic inputs shared by the tests of the first GNSS fix: a seeded
     * pseudo-random timestamp and user frame, and the error-free GNSS measurements of that user.
     *
     * @return test inputs.
     */
    private static Fixture createFixture() {
        final var randomizer = new UniformRandomizer(new Random(SEED));
        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var userFrame = createFrame(randomizer);
        final var satellites = createSatellites(randomizer, userFrame);
        final var measurements = generateMeasurements(timeSeconds, satellites, userFrame, createGnssConfig(0.0, 0.0),
                new Random(SEED));
        return new Fixture(timeSeconds, userFrame, measurements);
    }

    /**
     * Creates a frame with a seeded pseudo-random position, velocity and attitude.
     *
     * @param randomizer seeded randomizer.
     * @return a new ECEF frame.
     */
    private static ECEFFrame createFrame(final UniformRandomizer randomizer) {
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

        try {
            return new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            // never happens: transformation is always body to ECEF
            throw new IllegalStateException(e);
        }
    }

    /**
     * Creates a constellation of satellites in view of a user, as the existing GNSS tests do but with
     * a seeded randomizer: one satellite per azimuth sector, at a seeded pseudo-random azimuth within
     * the sector and elevation above the masking angle, at a typical GPS range and with a seeded
     * pseudo-random velocity. Spreading azimuths and elevations keeps the geometry well conditioned.
     *
     * @param randomizer seeded randomizer.
     * @param userFrame  frame containing the user position.
     * @return positions and velocities of the satellites at time zero.
     */
    private static List<ECEFPositionAndVelocity> createSatellites(
            final UniformRandomizer randomizer, final ECEFFrame userFrame) {
        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(userFrame);
        // rows of the ECEF to NED matrix are the north, east and down axes resolved in ECEF axes
        final var cen = CoordinateTransformation.ecefToNedMatrix(nedFrame.getLatitude(), nedFrame.getLongitude());
        final var sector = 2.0 * Math.PI / NUM_SATELLITES;

        final var result = new ArrayList<ECEFPositionAndVelocity>();
        for (var i = 0; i < NUM_SATELLITES; i++) {
            final var azimuth = randomizer.nextDouble(i * sector, (i + 1) * sector);
            final var elevation = Math.toRadians(randomizer.nextDouble(MIN_SATELLITE_ELEVATION_DEGREES,
                    MAX_SATELLITE_ELEVATION_DEGREES));

            // line of sight resolved in NED axes
            final var north = Math.cos(elevation) * Math.cos(azimuth);
            final var east = Math.cos(elevation) * Math.sin(azimuth);
            final var down = -Math.sin(elevation);

            final var x = userFrame.getX() + SATELLITE_RANGE * (cen.getElementAt(0, 0) * north
                    + cen.getElementAt(1, 0) * east + cen.getElementAt(2, 0) * down);
            final var y = userFrame.getY() + SATELLITE_RANGE * (cen.getElementAt(0, 1) * north
                    + cen.getElementAt(1, 1) * east + cen.getElementAt(2, 1) * down);
            final var z = userFrame.getZ() + SATELLITE_RANGE * (cen.getElementAt(0, 2) * north
                    + cen.getElementAt(1, 2) * east + cen.getElementAt(2, 2) * down);

            final var vx = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);

            result.add(new ECEFPositionAndVelocity(x, y, z, vx, vy, vz));
        }
        return result;
    }

    /**
     * Creates the configuration used to generate GNSS measurements. Only white tracking errors are
     * configured: there are no slowly varying range biases (signal in space, ionosphere and
     * troposphere errors are zero), so that the errors of consecutive fixes are uncorrelated, as the
     * Kalman filter assumes.
     *
     * @param codeTrackingSD      code tracking error standard deviation expressed in meters. Zero
     *                            generates error-free pseudo-ranges.
     * @param rangeRateTrackingSD range rate tracking error standard deviation expressed in meters per
     *                            second. Zero generates error-free pseudo-range rates.
     * @return GNSS configuration.
     */
    private static GNSSConfig createGnssConfig(final double codeTrackingSD, final double rangeRateTrackingSD) {
        final var result = new GNSSConfig();
        result.setNumberOfSatellites(NUM_SATELLITES);
        result.setMaskAngleDegrees(MASK_ANGLE_DEGREES);
        result.setSISErrorSD(0.0);
        result.setZenithIonosphereErrorSD(0.0);
        result.setZenithTroposphereErrorSD(0.0);
        result.setCodeTrackingErrorSD(codeTrackingSD);
        result.setRangeRateTrackingErrorSD(rangeRateTrackingSD);
        result.setInitialReceiverClockOffset(RECEIVER_CLOCK_OFFSET);
        result.setInitialReceiverClockDrift(RECEIVER_CLOCK_DRIFT);
        return result;
    }

    /**
     * Generates the GNSS measurements of a user at a given time, using the measurement generator the
     * existing GNSS tests use. Satellites move at constant velocity from their position at time zero.
     *
     * @param time       time expressed in seconds.
     * @param satellites positions and velocities of the satellites at time zero.
     * @param userFrame  frame containing the true user position and velocity at the given time.
     * @param config     GNSS configuration.
     * @param random     seeded random number generator.
     * @return one measurement per satellite.
     */
    private static List<GNSSMeasurement> generateMeasurements(
            final double time, final List<ECEFPositionAndVelocity> satellites, final ECEFFrame userFrame,
            final GNSSConfig config, final Random random) {
        final var movedSatellites = new ArrayList<ECEFPositionAndVelocity>();
        for (final var satellite : satellites) {
            movedSatellites.add(new ECEFPositionAndVelocity(
                    satellite.getX() + satellite.getVx() * time, satellite.getY() + satellite.getVy() * time,
                    satellite.getZ() + satellite.getVz() * time, satellite.getVx(), satellite.getVy(),
                    satellite.getVz()));
        }

        final var biases = Collections.nCopies(satellites.size(), 0.0);
        final var user = new ECEFPositionAndVelocity(userFrame.getECEFPosition(), userFrame.getECEFVelocity());
        final var result = new ArrayList<>(GNSSMeasurementsGenerator.generate(time, movedSatellites, user, biases,
                config, random));

        // every satellite is above the masking angle
        assertEquals(satellites.size(), result.size());
        return result;
    }

    /**
     * Creates the deterministic Kalman filter configuration used by the tests (tactical grade values
     * of Groves' {@code INS_GNSS_Demo_3}).
     *
     * @return Kalman filter configuration.
     */
    private static INSLooselyCoupledKalmanConfig createKalmanConfig() {
        return new INSLooselyCoupledKalmanConfig(GYRO_NOISE_PSD, ACCELEROMETER_NOISE_PSD, ACCELEROMETER_BIAS_PSD,
                GYRO_BIAS_PSD, FIX_POSITION_SD, FIX_VELOCITY_SD);
    }

    /**
     * Creates the deterministic initial uncertainties used by the tests (values of Groves'
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
     * Creates an estimator for a trajectory, with the true initial attitude, and initialises its
     * position and velocity with error-free GNSS measurements of the true initial position, provided
     * once at time zero.
     *
     * @param trajectory    trajectory to be followed.
     * @param epochInterval minimum interval between Kalman filter epochs expressed in seconds.
     * @return initialised estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException never thrown.
     * @throws LockedException                               if estimator is locked.
     * @throws NotReadyException                             if estimator is not ready.
     * @throws INSGNSSException                              if estimation fails.
     */
    private static INSGNSSLooselyCoupledKalmanFilteredEstimator createInitializedEstimator(
            final KalmanFilteredEstimatorTrajectory trajectory, final double epochInterval)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            INSGNSSException {
        final var initialFrame = trajectory.getInitialFrame();
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(), epochInterval,
                createInitConfig(), initialFrame.getCoordinateTransformation());

        final var satellites = createSatellites(new UniformRandomizer(new Random(SEED)), initialFrame);
        final var measurements = generateMeasurements(0.0, satellites, initialFrame, createGnssConfig(0.0, 0.0),
                new Random(SEED));
        assertTrue(estimator.updateMeasurements(measurements, 0.0));

        final var state = estimator.getState();
        assertNotNull(state);
        assertEquals(0.0, positionError(initialFrame, state.getFrame()), GNSS_POSITION_TOLERANCE);
        assertEquals(0.0, velocityError(initialFrame, state.getFrame()), GNSS_VELOCITY_TOLERANCE);
        return estimator;
    }

    /**
     * Feeds a whole trajectory to an estimator: the first kinematics sample is provided at the start
     * time (which only initialises the time reference), then one sample at the end of each sample
     * interval.
     *
     * @param estimator  estimator to be fed.
     * @param trajectory trajectory providing the error-free kinematics.
     * @return number of samples accepted by the estimator.
     * @throws LockedException  if estimator is locked.
     * @throws INSGNSSException if estimation fails.
     */
    private static int feed(
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator,
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
     * Asserts that an estimator has not received any measurement yet.
     *
     * @param estimator estimator to be checked.
     */
    private void assertNotInitialized(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        assertEquals(0, updateGNSSMeasurementsStart);
        assertEquals(0, updateGNSSMeasurementsEnd);
        assertEquals(0, updateBodyKinematicsStart);
        assertEquals(0, updateBodyKinematicsEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);

        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getLastStateTimestamp());
    }

    /**
     * Provides an IMU sample to an estimator that has not received GNSS measurements yet, and asserts
     * that the sample is accepted and kept, that the attitude is the expected one, and that neither a
     * GNSS estimation nor a Kalman filter state are available before the first fix.
     *
     * @param estimator   estimator to be checked.
     * @param kinematics  IMU sample to be provided.
     * @param timeSeconds timestamp of the sample expressed in seconds.
     * @param expectedC   expected body to ECEF attitude.
     * @throws LockedException  if estimator is locked.
     * @throws INSGNSSException if estimation fails.
     */
    private static void assertKinematicsBeforeFirstFix(
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator, final BodyKinematics kinematics,
            final double timeSeconds, final CoordinateTransformation expectedC) throws LockedException,
            INSGNSSException {
        assertTrue(estimator.updateBodyKinematics(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertEquals(kinematics, estimator.getKinematics());
        final var kinematics2 = new BodyKinematics();
        assertTrue(estimator.getKinematics(kinematics2));
        assertEquals(kinematics, kinematics2);
        // no bias has been estimated, so corrected kinematics equal provided ones
        assertEquals(kinematics, estimator.getCorrectedKinematics());
        final var correctedKinematics = new BodyKinematics();
        assertTrue(estimator.getCorrectedKinematics(correctedKinematics));
        assertEquals(kinematics, correctedKinematics);

        // the first sample only sets the time reference, so attitude is the initial one
        assertEquals(expectedC, estimator.getCoordinateTransformation());
        final var c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(expectedC, c2);
    }

    /**
     * Asserts the outcome of the first GNSS measurements provided to an estimator: measurements are
     * kept, the least squares fix is the true position and velocity, a Kalman filter epoch has been
     * run at the timestamp of the measurements, and the INS solution has been initialised with the
     * fix while keeping the expected attitude.
     *
     * @param estimator   estimator to be checked.
     * @param fixture     inputs of the test.
     * @param timeSeconds timestamp of the measurements expressed in seconds.
     * @param expectedC   expected body to ECEF attitude.
     */
    private static void assertFirstFix(
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator, final Fixture fixture,
            final double timeSeconds, final CoordinateTransformation expectedC) {
        assertFalse(estimator.isRunning());
        assertEquals(fixture.measurements(), estimator.getMeasurements());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);

        // attitude is preserved
        assertTrue(estimator.getCoordinateTransformation().equals(expectedC, ABSOLUTE_ERROR));
        final var c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertTrue(expectedC.equals(c2, ABSOLUTE_ERROR));

        final var estimation1 = estimator.getEstimation();
        assertNotNull(estimation1);
        final var estimation2 = new GNSSEstimation();
        assertTrue(estimator.getEstimation(estimation2));
        assertEquals(estimation1, estimation2);

        final var state1 = estimator.getState();
        assertNotNull(state1);
        final var state2 = new INSLooselyCoupledKalmanState();
        assertTrue(estimator.getState(state2));
        assertEquals(state1, state2);

        assertFixEqualsTruth(fixture.userFrame(), estimation1);
        assertStateEqualsFix(estimation1, state1);
        assertTrue(state1.getFrame().getCoordinateTransformation().equals(expectedC, ABSOLUTE_ERROR));
    }

    /**
     * Asserts that a least squares GNSS fix obtained from error-free measurements is the true position
     * and velocity.
     *
     * @param truth      frame containing the true position and velocity.
     * @param estimation GNSS estimation to be checked.
     */
    private static void assertFixEqualsTruth(final ECEFFrame truth, final GNSSEstimation estimation) {
        assertNotNull(estimation);
        assertEquals(0.0, norm(estimation.getX() - truth.getX(), estimation.getY() - truth.getY(),
                estimation.getZ() - truth.getZ()), GNSS_POSITION_TOLERANCE);
        assertEquals(0.0, norm(estimation.getVx() - truth.getVx(), estimation.getVy() - truth.getVy(),
                estimation.getVz() - truth.getVz()), GNSS_VELOCITY_TOLERANCE);
    }

    /**
     * Asserts that the Kalman filter state holds the position and velocity of a GNSS fix. This is the
     * case right after the first fix: the INS solution is initialised with the fix, so the Kalman
     * filter epoch has a zero innovation and applies no correction.
     *
     * @param estimation GNSS estimation of the first fix.
     * @param state      Kalman filter state to be checked.
     */
    private static void assertStateEqualsFix(
            final GNSSEstimation estimation, final INSLooselyCoupledKalmanState state) {
        assertNotNull(state);
        final var frame = state.getFrame();
        assertEquals(0.0, norm(frame.getX() - estimation.getX(), frame.getY() - estimation.getY(),
                frame.getZ() - estimation.getZ()), CHAIN_POSITION_TOLERANCE);
        assertEquals(0.0, norm(frame.getVx() - estimation.getVx(), frame.getVy() - estimation.getVy(),
                frame.getVz() - estimation.getVz()), CHAIN_VELOCITY_TOLERANCE);
    }

    /**
     * Checks the behaviour of body kinematics updates after a first GNSS fix: the first sample only
     * sets the time reference (no navigation), a repeated timestamp is ignored, and the second sample
     * is integrated with the ECEF inertial navigation equations over the elapsed interval, without
     * running any Kalman filter epoch nor modifying the GNSS estimation.
     *
     * @param fixture    inputs of the test. The fix is provided at its timestamp, the first sample at
     *                   the same timestamp and the second one at twice that timestamp.
     * @param kinematics kinematics provided on both samples.
     * @return frames estimated right after the fix and after the second sample.
     * @throws LockedException            if estimator is locked.
     * @throws NotReadyException          if estimator is not ready.
     * @throws INSGNSSException           if estimation fails.
     * @throws InertialNavigatorException if the oracle navigation fails.
     */
    private IntegratedSample assertUpdateBodyKinematicsIntegratesSample(
            final Fixture fixture, final BodyKinematics kinematics) throws LockedException, NotReadyException,
            INSGNSSException, InertialNavigatorException {
        final var timeSeconds = fixture.timeSeconds();
        final var estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(createKalmanConfig(),
                createInitConfig(), this);

        reset();
        assertNotInitialized(estimator);
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));

        // update measurement
        assertTrue(estimator.updateMeasurements(fixture.measurements(), timeSeconds));

        final var estimation1 = estimator.getEstimation();
        final var state1 = estimator.getState();
        assertFixEqualsTruth(fixture.userFrame(), estimation1);
        assertStateEqualsFix(estimation1, state1);
        final var initialFrame = state1.getFrame();
        // without initial attitude, body axes are aligned with ECEF axes
        assertTrue(initialFrame.getCoordinateTransformation().equals(new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME), ABSOLUTE_ERROR));

        // first sample only sets the time reference: no navigation and no Kalman filter epoch
        assertTrue(estimator.updateBodyKinematics(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(kinematics, estimator.getKinematics());
        // no bias has been estimated (the first epoch has a zero innovation)
        assertEquals(kinematics, estimator.getCorrectedKinematics());
        assertFrameEquals(initialFrame, estimator.getState().getFrame());

        // a sample with the same timestamp makes no action
        final var state2 = estimator.getState();
        assertFalse(estimator.updateBodyKinematics(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));

        assertFalse(estimator.isRunning());
        assertEquals(state2, estimator.getState());

        // a sample with a new timestamp is integrated over the elapsed interval
        assertTrue(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(fixture.measurements(), estimator.getMeasurements());
        assertEquals(kinematics, estimator.getKinematics());
        final var kinematics2 = new BodyKinematics();
        assertTrue(estimator.getKinematics(kinematics2));
        assertEquals(kinematics, kinematics2);
        assertEquals(kinematics, estimator.getCorrectedKinematics());

        final var expected = KalmanFilteredEstimatorTrajectory.chainNavigator(initialFrame, List.of(kinematics),
                timeSeconds);
        final var state3 = estimator.getState();
        final var result = state3.getFrame();
        assertFrameEquals(expected, result);

        // attitude is the integrated one
        assertTrue(estimator.getCoordinateTransformation().getMatrix().equals(
                expected.getCoordinateTransformationMatrix(), CHAIN_ROTATION_TOLERANCE));

        // no Kalman filter epoch has been run: its timestamp and covariance are the ones of the fix,
        // and the GNSS estimation is not replaced by the INS solution
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        assertEquals(state1.getCovariance(), state3.getCovariance());
        assertEquals(estimation1, estimator.getEstimation());

        assertEquals(1, updateGNSSMeasurementsStart);
        assertEquals(1, updateGNSSMeasurementsEnd);
        assertEquals(2, updateBodyKinematicsStart);
        assertEquals(2, updateBodyKinematicsEnd);
        assertEquals(1, propagateStart);
        assertEquals(1, propagateEnd);

        return new IntegratedSample(initialFrame, result);
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

    /**
     * Solution of an estimator before and after integrating an IMU sample.
     *
     * @param initialFrame frame estimated right after the first GNSS fix.
     * @param finalFrame   frame estimated after integrating the sample.
     */
    private record IntegratedSample(ECEFFrame initialFrame, ECEFFrame finalFrame) {
    }

    /**
     * Deterministic inputs shared by the tests of the first GNSS fix.
     *
     * @param timeSeconds  timestamp of the measurements expressed in seconds.
     * @param userFrame    frame containing the true user position, velocity and attitude.
     * @param measurements error-free GNSS measurements of the user.
     */
    private record Fixture(double timeSeconds, ECEFFrame userFrame, List<GNSSMeasurement> measurements) {

        /**
         * Constructor.
         *
         * @param timeSeconds  timestamp of the measurements expressed in seconds.
         * @param userFrame    frame containing the true user position, velocity and attitude.
         * @param measurements error-free GNSS measurements of the user.
         */
        private Fixture {
            measurements = List.copyOf(measurements);
        }
    }
}
