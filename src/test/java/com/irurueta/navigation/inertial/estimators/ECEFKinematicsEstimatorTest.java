/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RankDeficientMatrixException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static com.irurueta.navigation.frames.CoordinateTransformation.ROWS;
import static org.junit.jupiter.api.Assertions.*;

class ECEFKinematicsEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double MAX_ANGLE_VARIATION_DEGREES = 5.0;

    /**
     * Time interval, expressed in seconds, of the second pass of the randomised tests. It is long enough for
     * second-order terms (Earth rotation during the interval, eq. (5.85)) to be observable.
     */
    private static final double LONG_TIME_INTERVAL_SECONDS = 1.0;

    /**
     * Maximum absolute attitude variation per Euler angle, expressed in degrees, of the second pass of the
     * randomised tests. It is large enough for second-order attitude terms to be observable.
     */
    private static final double LARGE_MAX_ANGLE_VARIATION_DEGREES = 45.0;

    /**
     * Seed of the randomizers used by the tests, fixed so that tests are deterministic.
     */
    private static final long SEED = 36L;

    /**
     * Number of randomly generated pairs of frames checked on each pass of the randomised tests.
     */
    private static final int TIMES = 1000;

    private static final double MIN_POSITION_VARIATION_DEGREES = -1e-4;
    private static final double MAX_POSITION_VARIATION_DEGREES = 1e-4;

    private static final double MIN_HEIGHT_VARIATION = -0.5;
    private static final double MAX_HEIGHT_VARIATION = 0.5;

    private static final double MIN_VELOCITY_VARIATION = -0.1;
    private static final double MAX_VELOCITY_VARIATION = 0.1;

    private static final double SCALING_THRESHOLD = 2e-5;
    private static final double ALPHA_THRESHOLD = 1e-8;

    /**
     * Maximum allowed difference between the estimator and the in-test reference implementation of
     * {@code Kinematics_ECEF.m}. Both perform the same arithmetic in a different order of operations, so they
     * only differ by floating point rounding: the largest difference observed over 2000 seeded pairs of frames
     * per pass is 2.0e-14 (time interval of 0.02 s) and 7.6e-15 (time interval of 1 s). 1e-9 leaves margin for
     * rounding while being three orders of magnitude below the smallest algorithmic error fixed in the estimator
     * (7.8e-7 m/s^2).
     */
    private static final double ABSOLUTE_ERROR = 1e-9;

    /**
     * Maximum allowed difference in specific force, expressed in meters per squared second, between the ECEF, ECI
     * and NED kinematics estimators. It is the smallest power of ten satisfied by the three algorithms over the
     * seeded, kinematically consistent pairs of frames of the comparison test, where the largest observed
     * differences are 9.9e-4 (ECEF vs. NED), 1.0e-3 (ECI vs. NED) and 2.9e-5 (ECI vs. ECEF).
     */
    private static final double COMPARE_SPECIFIC_FORCE_ERROR = 1e-2;

    /**
     * Maximum allowed difference in angular rate, expressed in radians per second, between the ECEF, ECI and NED
     * kinematics estimators. It is the smallest power of ten satisfied by the three algorithms over the seeded,
     * kinematically consistent pairs of frames of the comparison test, where the largest observed difference is
     * 1.5e-9 (both ECEF vs. NED and ECI vs. NED).
     */
    private static final double COMPARE_ANGULAR_RATE_ERROR = 1e-8;

    /**
     * Factor used to average the velocities at the start and the end of a time interval.
     */
    private static final double MEAN_FACTOR = 0.5;

    /**
     * Time intervals, expressed in seconds, used by the navigator round trip tests.
     */
    private static final double[] ROUND_TRIP_TIME_INTERVALS = {0.1, 1.0};

    /**
     * Longest time interval, expressed in seconds, used by the navigator round trip tests. With the round trip
     * velocity, the positions at the start and at the end of this interval are about 36 meters apart.
     */
    private static final double ROUND_TRIP_LONG_TIME_INTERVAL = 1.0;

    /**
     * North velocity of the round trip body at the start of the interval, expressed in meters per second.
     */
    private static final double ROUND_TRIP_VN = 30.0;

    /**
     * East velocity of the round trip body at the start of the interval, expressed in meters per second.
     */
    private static final double ROUND_TRIP_VE = 20.0;

    /**
     * Down velocity of the round trip body at the start of the interval, expressed in meters per second.
     */
    private static final double ROUND_TRIP_VD = -1.0;

    /**
     * Roll angle of the round trip body at the start of the interval, expressed in degrees.
     */
    private static final double ROUND_TRIP_ROLL_DEGREES = 10.0;

    /**
     * Pitch angle of the round trip body at the start of the interval, expressed in degrees.
     */
    private static final double ROUND_TRIP_PITCH_DEGREES = -20.0;

    /**
     * Yaw angle of the round trip body at the start of the interval, expressed in degrees.
     */
    private static final double ROUND_TRIP_YAW_DEGREES = 30.0;

    /**
     * X coordinate of the round trip specific force, expressed in meters per squared second.
     */
    private static final double ROUND_TRIP_FX = 1.0;

    /**
     * Y coordinate of the round trip specific force, expressed in meters per squared second.
     */
    private static final double ROUND_TRIP_FY = 2.0;

    /**
     * Z coordinate of the round trip specific force, expressed in meters per squared second.
     */
    private static final double ROUND_TRIP_FZ = -9.8;

    /**
     * X coordinate of the round trip angular rate, expressed in radians per second.
     */
    private static final double ROUND_TRIP_ANGULAR_RATE_X = 0.5;

    /**
     * Y coordinate of the round trip angular rate, expressed in radians per second.
     */
    private static final double ROUND_TRIP_ANGULAR_RATE_Y = -0.3;

    /**
     * Z coordinate of the round trip angular rate, expressed in radians per second.
     */
    private static final double ROUND_TRIP_ANGULAR_RATE_Z = 1.0;

    /**
     * Maximum allowed difference between the kinematics fed to {@link ECEFInertialNavigator} and the kinematics
     * recovered by {@link ECEFKinematicsEstimator} from the navigated frame. Navigator and estimator implement
     * exactly inverse equations (Groves eq. (5.73)-(5.85) and (5.36)), so the only expected difference is
     * floating point rounding: a rounding of about 1e-16 relative in velocities of about 4e2 m/s divided by a
     * time interval of 0.1 s gives about 1e-12 m/s^2, hence 1e-9 leaves three orders of magnitude of margin while
     * remaining far below the smallest algorithmic error this test must detect (7.8e-7 m/s^2).
     */
    private static final double ROUND_TRIP_ERROR = 1e-9;

    /**
     * Minimum difference in specific force, expressed in meters per squared second, that must be observed when
     * gravity is evaluated at the end instead of at the start of the round trip long time interval. Positions are
     * about 36 m apart and the gravity gradient is about 3e-6 s^-2 vertically and 1.5e-6 s^-2 horizontally, which
     * yields a difference of about 1e-4 m/s^2; 1e-6 is two orders of magnitude below that and three orders
     * of magnitude above {@link #ROUND_TRIP_ERROR}.
     */
    private static final double MIN_POSITION_SENSITIVITY = 1e-6;

    /**
     * Checks that all instance {@code estimate} overloads are consistent with each other, both for a short time
     * interval with small attitude changes and for a long time interval with large attitude changes.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    @Test
    void testEstimate() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {
        final var randomizer = createRandomizer();
        for (var t = 0; t < TIMES; t++) {
            checkEstimate(randomizer, TIME_INTERVAL_SECONDS, MAX_ANGLE_VARIATION_DEGREES);
            checkEstimate(randomizer, LONG_TIME_INTERVAL_SECONDS, LARGE_MAX_ANGLE_VARIATION_DEGREES);
        }
    }

    /**
     * Checks that all instance {@code estimateAndReturnNew} overloads are consistent with each other, both for a short
     * time interval with small attitude changes and for a long time interval with large attitude changes.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    @Test
    void testEstimateAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {
        final var randomizer = createRandomizer();
        for (var t = 0; t < TIMES; t++) {
            checkEstimateAndReturnNew(randomizer, TIME_INTERVAL_SECONDS, MAX_ANGLE_VARIATION_DEGREES);
            checkEstimateAndReturnNew(randomizer, LONG_TIME_INTERVAL_SECONDS, LARGE_MAX_ANGLE_VARIATION_DEGREES);
        }
    }

    /**
     * Checks that all static {@code estimateKinematics} overloads are consistent with each other and with the in-test
     * reference implementation, both for a short time interval with small attitude changes and for a long time
     * interval with large attitude changes.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws WrongSizeException                            never happens.
     * @throws RankDeficientMatrixException                  never happens.
     * @throws DecomposerException                           never happens.
     */
    @Test
    void testEstimateKinematics() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException,
            WrongSizeException, RankDeficientMatrixException, DecomposerException  {
        final var randomizer = createRandomizer();
        for (var t = 0; t < TIMES; t++) {
            checkEstimateKinematics(randomizer, TIME_INTERVAL_SECONDS, MAX_ANGLE_VARIATION_DEGREES);
            checkEstimateKinematics(randomizer, LONG_TIME_INTERVAL_SECONDS, LARGE_MAX_ANGLE_VARIATION_DEGREES);
        }
    }

    /**
     * Checks that all static {@code estimateKinematicsAndReturnNew} overloads are consistent with each other and with
     * the in-test reference implementation, both for a short time interval with small attitude changes and for a long
     * time interval with large attitude changes.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws WrongSizeException                            never happens.
     * @throws RankDeficientMatrixException                  never happens.
     * @throws DecomposerException                           never happens.
     */
    @Test
    void testEstimateKinematicsAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException, DecomposerException  {
        final var randomizer = createRandomizer();
        for (var t = 0; t < TIMES; t++) {
            checkEstimateKinematicsAndReturnNew(randomizer, TIME_INTERVAL_SECONDS, MAX_ANGLE_VARIATION_DEGREES);
            checkEstimateKinematicsAndReturnNew(randomizer, LONG_TIME_INTERVAL_SECONDS,
                    LARGE_MAX_ANGLE_VARIATION_DEGREES);
        }
    }

    @Test
    void testEstimateKinematicsWhenNegativeIntervalThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame(createRandomizer());
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame, createRandomizer(), MAX_ANGLE_VARIATION_DEGREES);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final var c = newEcefFrame.getCoordinateTransformation();
        final var oldC = oldEcefFrame.getCoordinateTransformation();

        final var vx = newEcefFrame.getVx();
        final var vy = newEcefFrame.getVy();
        final var vz = newEcefFrame.getVz();

        final var oldVx = oldEcefFrame.getVx();
        final var oldVy = oldEcefFrame.getVy();
        final var oldVz = oldEcefFrame.getVz();

        final var x = newEcefFrame.getX();
        final var y = newEcefFrame.getY();
        final var z = newEcefFrame.getZ();

        assertThrows(IllegalArgumentException.class, () -> ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                -1.0, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z));
    }

    @Test
    void testEstimateKinematicsWhenInvalidCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame(createRandomizer());
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame, createRandomizer(), MAX_ANGLE_VARIATION_DEGREES);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final var c = newEcefFrame.getCoordinateTransformation();
        final var oldC = oldEcefFrame.getCoordinateTransformation();

        final var vx = newEcefFrame.getVx();
        final var vy = newEcefFrame.getVy();
        final var vz = newEcefFrame.getVz();

        final var oldVx = oldEcefFrame.getVx();
        final var oldVy = oldEcefFrame.getVy();
        final var oldVz = oldEcefFrame.getVz();

        final var x = newEcefFrame.getX();
        final var y = newEcefFrame.getY();
        final var z = newEcefFrame.getZ();

        c.setDestinationType(FrameType.BODY_FRAME);
        assertThrows(IllegalArgumentException.class, () -> ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z));
    }

    @Test
    void testEstimateKinematicsWhenInvalidOldCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame(createRandomizer());
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame, createRandomizer(), MAX_ANGLE_VARIATION_DEGREES);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final var c = newEcefFrame.getCoordinateTransformation();
        final var oldC = oldEcefFrame.getCoordinateTransformation();

        final var vx = newEcefFrame.getVx();
        final var vy = newEcefFrame.getVy();
        final var vz = newEcefFrame.getVz();

        final var oldVx = oldEcefFrame.getVx();
        final var oldVy = oldEcefFrame.getVy();
        final var oldVz = oldEcefFrame.getVz();

        final var x = newEcefFrame.getX();
        final var y = newEcefFrame.getY();
        final var z = newEcefFrame.getZ();

        oldC.setDestinationType(FrameType.BODY_FRAME);
        assertThrows(IllegalArgumentException.class, () -> ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z));
    }

    @Test
    void testEstimateKinematicsWhenZeroTimeIntervalReturnsZeroValues() 
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        final var oldNedFrame = createOldNedFrame(createRandomizer());
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame, createRandomizer(), MAX_ANGLE_VARIATION_DEGREES);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final var c = newEcefFrame.getCoordinateTransformation();
        final var oldC = oldEcefFrame.getCoordinateTransformation();

        final var vx = newEcefFrame.getVx();
        final var vy = newEcefFrame.getVy();
        final var vz = newEcefFrame.getVz();

        final var oldVx = oldEcefFrame.getVx();
        final var oldVy = oldEcefFrame.getVy();
        final var oldVz = oldEcefFrame.getVz();

        final var x = newEcefFrame.getX();
        final var y = newEcefFrame.getY();
        final var z = newEcefFrame.getZ();

        final var k = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(0.0, c, oldC, vx, vy, vz, 
                oldVx, oldVy, oldVz, x, y, z);

        assertEquals(0.0, k.getFx(), 0.0);
        assertEquals(0.0, k.getFy(), 0.0);
        assertEquals(0.0, k.getFz(), 0.0);

        assertEquals(0.0, k.getAngularRateX(), 0.0);
        assertEquals(0.0, k.getAngularRateY(), 0.0);
        assertEquals(0.0, k.getAngularRateZ(), 0.0);

        final var k2 = estimateKinematics(0.0, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        assertTrue(k2.equals(k, 0.0));
    }

    /**
     * Checks that the ECEF, ECI and NED kinematics estimators agree for kinematically consistent pairs of frames
     * (the position at the end of the time interval is the position at the start plus the mean velocity times the
     * time interval, and ECI frames are obtained at the epochs of the start and the end of the time interval).
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    @Test
    void testCompareKinematics() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {
        final var randomizer = createRandomizer();
        for (var t = 0; t < TIMES; t++) {
            final var oldNedFrame = createOldNedFrame(randomizer);
            final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
            final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(0.0, oldEcefFrame);

            // randomly generated attitude and velocity at the end of the time interval, with a position that is
            // consistent with the velocities at the start and the end of the time interval (5.38)
            final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                    createNewNedFrame(oldNedFrame, randomizer, MAX_ANGLE_VARIATION_DEGREES));
            newEcefFrame.setCoordinates(
                    oldEcefFrame.getX() + MEAN_FACTOR * (oldEcefFrame.getVx() + newEcefFrame.getVx())
                            * TIME_INTERVAL_SECONDS,
                    oldEcefFrame.getY() + MEAN_FACTOR * (oldEcefFrame.getVy() + newEcefFrame.getVy())
                            * TIME_INTERVAL_SECONDS,
                    oldEcefFrame.getZ() + MEAN_FACTOR * (oldEcefFrame.getVz() + newEcefFrame.getVz())
                            * TIME_INTERVAL_SECONDS);
            final var newNedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(newEcefFrame);
            final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                    newEcefFrame);

            final var nedK = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newNedFrame,
                    oldNedFrame);
            final var ecefK = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    newEcefFrame, oldEcefFrame);
            final var eciK = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame,
                    oldEciFrame);

            final var nedSpecificForceNorm = nedK.getSpecificForceNorm();
            final var ecefSpecificForceNorm = ecefK.getSpecificForceNorm();
            final var eciSpecificForceNorm = eciK.getSpecificForceNorm();

            final var nedAngularRateNorm = nedK.getAngularRateNorm();
            final var ecefAngularRateNorm = ecefK.getAngularRateNorm();
            final var eciAngularRateNorm = eciK.getAngularRateNorm();

            assertEquals(ecefSpecificForceNorm, nedSpecificForceNorm, COMPARE_SPECIFIC_FORCE_ERROR);
            assertEquals(eciSpecificForceNorm, nedSpecificForceNorm, COMPARE_SPECIFIC_FORCE_ERROR);

            assertEquals(ecefAngularRateNorm, nedAngularRateNorm, COMPARE_ANGULAR_RATE_ERROR);
            assertEquals(eciAngularRateNorm, nedAngularRateNorm, COMPARE_ANGULAR_RATE_ERROR);

            assertEquals(ecefK.getFx(), nedK.getFx(), COMPARE_SPECIFIC_FORCE_ERROR);
            assertEquals(ecefK.getFy(), nedK.getFy(), COMPARE_SPECIFIC_FORCE_ERROR);
            assertEquals(ecefK.getFz(), nedK.getFz(), COMPARE_SPECIFIC_FORCE_ERROR);
            assertEquals(ecefK.getAngularRateX(), nedK.getAngularRateX(), COMPARE_ANGULAR_RATE_ERROR);
            assertEquals(ecefK.getAngularRateY(), nedK.getAngularRateY(), COMPARE_ANGULAR_RATE_ERROR);
            assertEquals(ecefK.getAngularRateZ(), nedK.getAngularRateZ(), COMPARE_ANGULAR_RATE_ERROR);

            assertEquals(eciK.getFx(), nedK.getFx(), COMPARE_SPECIFIC_FORCE_ERROR);
            assertEquals(eciK.getFy(), nedK.getFy(), COMPARE_SPECIFIC_FORCE_ERROR);
            assertEquals(eciK.getFz(), nedK.getFz(), COMPARE_SPECIFIC_FORCE_ERROR);
            assertEquals(eciK.getAngularRateX(), nedK.getAngularRateX(), COMPARE_ANGULAR_RATE_ERROR);
            assertEquals(eciK.getAngularRateY(), nedK.getAngularRateY(), COMPARE_ANGULAR_RATE_ERROR);
            assertEquals(eciK.getAngularRateZ(), nedK.getAngularRateZ(), COMPARE_ANGULAR_RATE_ERROR);
        }
    }

    /**
     * Checks that the kinematics fed to {@link ECEFInertialNavigator} are recovered by the estimator from the
     * navigated frame when the body rotates and gravity is evaluated at the position at the start of the interval.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws InertialNavigatorException                    never happens.
     */
    @Test
    void testNavigatorRoundTripWithRotation() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, InertialNavigatorException {

        final var oldFrame = createRoundTripOldFrame();
        final var k = createRoundTripKinematics();

        for (final var timeInterval : ROUND_TRIP_TIME_INTERVALS) {
            final var newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldFrame, k);

            final var result = new BodyKinematics();
            ECEFKinematicsEstimator.estimateKinematics(timeInterval, newFrame.getCoordinateTransformation(),
                    oldFrame.getCoordinateTransformation(), newFrame.getVx(), newFrame.getVy(), newFrame.getVz(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(),
                    oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(), result);

            assertTrue(result.equals(k, ROUND_TRIP_ERROR), "time interval: " + timeInterval
                    + " s, specific force error: " + specificForceDistance(result, k)
                    + " m/s^2, angular rate error: " + angularRateDistance(result, k) + " rad/s");
        }
    }

    /**
     * Checks that every overload receiving both the current and the previous frame evaluates gravity at the
     * position of the previous frame (start of the time interval), so that it recovers the kinematics fed to
     * {@link ECEFInertialNavigator}.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws InertialNavigatorException                    never happens.
     */
    @Test
    void testFrameOverloadsUseOldPosition() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, InertialNavigatorException {

        final var oldFrame = createRoundTripOldFrame();
        final var k = createRoundTripKinematics();
        final var newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(ROUND_TRIP_LONG_TIME_INTERVAL, oldFrame,
                k);

        final var c = newFrame.getCoordinateTransformation();
        final var oldC = oldFrame.getCoordinateTransformation();

        // reference results obtained with an explicit position
        final var oldPositionK = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(ROUND_TRIP_LONG_TIME_INTERVAL, c, oldC,
                newFrame.getVx(), newFrame.getVy(), newFrame.getVz(),
                oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(),
                oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(), oldPositionK);

        final var newPositionK = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(ROUND_TRIP_LONG_TIME_INTERVAL, c, oldC,
                newFrame.getVx(), newFrame.getVy(), newFrame.getVz(),
                oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(),
                newFrame.getX(), newFrame.getY(), newFrame.getZ(), newPositionK);

        // overloads receiving both frames
        final var timeInterval = new Time(ROUND_TRIP_LONG_TIME_INTERVAL, TimeUnit.SECOND);
        final var estimator = new ECEFKinematicsEstimator();

        final var k1 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(ROUND_TRIP_LONG_TIME_INTERVAL, newFrame, oldFrame, k1);

        final var k2 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newFrame, oldFrame, k2);

        final var k3 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(ROUND_TRIP_LONG_TIME_INTERVAL,
                newFrame, oldFrame);

        final var k4 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newFrame, oldFrame);

        final var k5 = new BodyKinematics();
        estimator.estimate(ROUND_TRIP_LONG_TIME_INTERVAL, newFrame, oldFrame, k5);

        final var k6 = new BodyKinematics();
        estimator.estimate(timeInterval, newFrame, oldFrame, k6);

        final var k7 = estimator.estimateAndReturnNew(ROUND_TRIP_LONG_TIME_INTERVAL, newFrame, oldFrame);

        final var k8 = estimator.estimateAndReturnNew(timeInterval, newFrame, oldFrame);

        for (final var result : new BodyKinematics[]{k1, k2, k3, k4, k5, k6, k7, k8}) {
            final var message = "specific force error: " + specificForceDistance(result, k) + " m/s^2";
            assertEquals(oldPositionK, result, message);
            assertTrue(result.equals(k, ROUND_TRIP_ERROR), message);
            assertTrue(specificForceDistance(result, newPositionK) > MIN_POSITION_SENSITIVITY);
        }
    }

    /**
     * Checks that all instance {@code estimate} overloads are consistent with each other for a randomly generated pair
     * of frames.
     *
     * @param randomizer               randomizer used to generate the frames.
     * @param timeIntervalSeconds      time interval between frames expressed in seconds.
     * @param maxAngleVariationDegrees maximum absolute attitude variation per Euler angle between frames,
     *                                 expressed in degrees.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private static void checkEstimate(
            final UniformRandomizer randomizer, final double timeIntervalSeconds,
            final double maxAngleVariationDegrees)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame(randomizer);
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame, randomizer, maxAngleVariationDegrees);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final var c = newEcefFrame.getCoordinateTransformation();
        final var oldC = oldEcefFrame.getCoordinateTransformation();

        final var vx = newEcefFrame.getVx();
        final var vy = newEcefFrame.getVy();
        final var vz = newEcefFrame.getVz();

        final var oldVx = oldEcefFrame.getVx();
        final var oldVy = oldEcefFrame.getVy();
        final var oldVz = oldEcefFrame.getVz();

        // position at the start of the time interval, where gravity must be evaluated
        final var x = oldEcefFrame.getX();
        final var y = oldEcefFrame.getY();
        final var z = oldEcefFrame.getZ();

        // position at the end of the time interval, used by overloads that only receive the new frame
        final var newX = newEcefFrame.getX();
        final var newY = newEcefFrame.getY();
        final var newZ = newEcefFrame.getZ();

        final var estimator = new ECEFKinematicsEstimator();

        final var k1 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k1);

        final var timeInterval = new Time(timeIntervalSeconds, TimeUnit.SECOND);
        final var k2 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final var velocity = new ECEFVelocity(vx, vy, vz);
        final var oldVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final var k3 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, velocity, oldVelocity, x, y, z, k3);

        final var k4 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, x, y, z, k4);

        final var ecefPosition = new ECEFPosition(x, y, z);
        final var k5 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition, k5);

        final var k6 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition, k6);

        final var k7 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, velocity, oldVelocity, ecefPosition, k7);

        final var k8 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, ecefPosition, k8);

        final var k9 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, newEcefFrame, oldC, oldVx, oldVy, oldVz, k9);

        final var k10 = new BodyKinematics();
        estimator.estimate(timeInterval, newEcefFrame, oldC, oldVx, oldVy, oldVz, k10);

        final var k11 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, newEcefFrame, oldC, oldVelocity, k11);

        final var k12 = new BodyKinematics();
        estimator.estimate(timeInterval, newEcefFrame, oldC, oldVelocity, k12);

        final var k13 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, newEcefFrame, oldEcefFrame, k13);

        final var k14 = new BodyKinematics();
        estimator.estimate(timeInterval, newEcefFrame, oldEcefFrame, k14);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var k15 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ,
                x, y, z, k15);

        final var k16 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z,
                k16);

        final var k17 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ,
                ecefPosition, k17);

        final var k18 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition,
                k18);

        final var k19 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, newEcefFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k19);

        final var k20 = new BodyKinematics();
        estimator.estimate(timeInterval, newEcefFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k20);

        final var position = new InhomogeneousPoint3D(x, y, z);
        final var k21 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, k21);

        final var k22 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, k22);

        final var k23 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, velocity, oldVelocity, position, k23);

        final var k24 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, position, k24);

        final var k25 = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ,
                position, k25);

        final var k26 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position,
                k26);

        final var newPositionK = new BodyKinematics();
        estimator.estimate(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, newX, newY, newZ,
                newPositionK);

        // overloads receiving an explicit position, or both the new and the old frame, evaluate
        // gravity at the position at the start of the time interval
        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);

        // overloads receiving only the new frame evaluate gravity at the position at the end of the
        // time interval
        assertEquals(newPositionK, k9);
        assertEquals(newPositionK, k10);
        assertEquals(newPositionK, k11);
        assertEquals(newPositionK, k12);
        assertEquals(newPositionK, k19);
        assertEquals(newPositionK, k20);
    }

    /**
     * Checks that all instance {@code estimateAndReturnNew} overloads are consistent with each other for a randomly
     * generated pair of frames.
     *
     * @param randomizer               randomizer used to generate the frames.
     * @param timeIntervalSeconds      time interval between frames expressed in seconds.
     * @param maxAngleVariationDegrees maximum absolute attitude variation per Euler angle between frames,
     *                                 expressed in degrees.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private static void checkEstimateAndReturnNew(
            final UniformRandomizer randomizer, final double timeIntervalSeconds,
            final double maxAngleVariationDegrees) throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame(randomizer);
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame, randomizer, maxAngleVariationDegrees);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final var c = newEcefFrame.getCoordinateTransformation();
        final var oldC = oldEcefFrame.getCoordinateTransformation();

        final var vx = newEcefFrame.getVx();
        final var vy = newEcefFrame.getVy();
        final var vz = newEcefFrame.getVz();

        final var oldVx = oldEcefFrame.getVx();
        final var oldVy = oldEcefFrame.getVy();
        final var oldVz = oldEcefFrame.getVz();

        // position at the start of the time interval, where gravity must be evaluated
        final var x = oldEcefFrame.getX();
        final var y = oldEcefFrame.getY();
        final var z = oldEcefFrame.getZ();

        // position at the end of the time interval, used by overloads that only receive the new frame
        final var newX = newEcefFrame.getX();
        final var newY = newEcefFrame.getY();
        final var newZ = newEcefFrame.getZ();

        final var estimator = new ECEFKinematicsEstimator();

        final var k1 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);

        final var timeInterval = new Time(timeIntervalSeconds, TimeUnit.SECOND);
        final var k2 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final var velocity = new ECEFVelocity(vx, vy, vz);
        final var oldVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final var k3 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, velocity, oldVelocity, x, y, z);

        final var k4 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity, x, y, z);

        final var ecefPosition = new ECEFPosition(x, y, z);
        final var k5 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                ecefPosition);

        final var k6 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                ecefPosition);

        final var k7 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, velocity, oldVelocity,
                ecefPosition);

        final var k8 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity, ecefPosition);

        final var k9 = estimator.estimateAndReturnNew(timeIntervalSeconds, newEcefFrame, oldC, oldVx, oldVy, oldVz);

        final var k10 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame, oldC, oldVx, oldVy, oldVz);

        final var k11 = estimator.estimateAndReturnNew(timeIntervalSeconds, newEcefFrame, oldC, oldVelocity);

        final var k12 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame, oldC, oldVelocity);

        final var k13 = estimator.estimateAndReturnNew(timeIntervalSeconds, newEcefFrame, oldEcefFrame);

        final var k14 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame, oldEcefFrame);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var k15 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final var k16 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final var k17 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition);

        final var k18 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition);

        final var k19 = estimator.estimateAndReturnNew(timeIntervalSeconds, newEcefFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final var k20 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final var position = new InhomogeneousPoint3D(x, y, z);
        final var k21 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position);

        final var k22 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position);

        final var k23 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, velocity, oldVelocity, position);

        final var k24 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity, position);

        final var k25 = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final var k26 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final var newPositionK = estimator.estimateAndReturnNew(timeIntervalSeconds, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, newX, newY, newZ);

        // overloads receiving an explicit position, or both the new and the old frame, evaluate
        // gravity at the position at the start of the time interval
        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);

        // overloads receiving only the new frame evaluate gravity at the position at the end of the
        // time interval
        assertEquals(newPositionK, k9);
        assertEquals(newPositionK, k10);
        assertEquals(newPositionK, k11);
        assertEquals(newPositionK, k12);
        assertEquals(newPositionK, k19);
        assertEquals(newPositionK, k20);
    }

    /**
     * Checks that all static {@code estimateKinematics} overloads are consistent with each other and with the in-test
     * reference implementation for a randomly generated pair of frames.
     *
     * @param randomizer               randomizer used to generate the frames.
     * @param timeIntervalSeconds      time interval between frames expressed in seconds.
     * @param maxAngleVariationDegrees maximum absolute attitude variation per Euler angle between frames,
     *                                 expressed in degrees.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws WrongSizeException                            never happens.
     * @throws RankDeficientMatrixException                  never happens.
     * @throws DecomposerException                           never happens.
     */
    private static void checkEstimateKinematics(
            final UniformRandomizer randomizer, final double timeIntervalSeconds,
            final double maxAngleVariationDegrees)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException,
            WrongSizeException, RankDeficientMatrixException, DecomposerException {

        final var oldNedFrame = createOldNedFrame(randomizer);
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame, randomizer, maxAngleVariationDegrees);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final var c = newEcefFrame.getCoordinateTransformation();
        final var oldC = oldEcefFrame.getCoordinateTransformation();

        final var vx = newEcefFrame.getVx();
        final var vy = newEcefFrame.getVy();
        final var vz = newEcefFrame.getVz();

        final var oldVx = oldEcefFrame.getVx();
        final var oldVy = oldEcefFrame.getVy();
        final var oldVz = oldEcefFrame.getVz();

        // position at the start of the time interval, where gravity must be evaluated
        final var x = oldEcefFrame.getX();
        final var y = oldEcefFrame.getY();
        final var z = oldEcefFrame.getZ();

        // position at the end of the time interval, used by overloads that only receive the new frame
        final var newX = newEcefFrame.getX();
        final var newY = newEcefFrame.getY();
        final var newZ = newEcefFrame.getZ();

        final var k1 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, k1);

        final var timeInterval = new Time(timeIntervalSeconds, TimeUnit.SECOND);
        final var k2 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final var velocity = new ECEFVelocity(vx, vy, vz);
        final var oldVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final var k3 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, velocity, oldVelocity, x, y, z, k3);

        final var k4 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, x, y, z, k4);

        final var ecefPosition = new ECEFPosition(x, y, z);
        final var k5 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                ecefPosition, k5);

        final var k6 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition,
                k6);

        final var k7 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, velocity, oldVelocity, ecefPosition,
                k7);

        final var k8 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, ecefPosition, k8);

        final var k9 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, newEcefFrame, oldC, oldVx, oldVy, oldVz, k9);

        final var k10 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame, oldC, oldVx, oldVy, oldVz, k10);

        final var k11 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, newEcefFrame, oldC, oldVelocity, k11);

        final var k12 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame, oldC, oldVelocity, k12);

        final var k13 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, newEcefFrame, oldEcefFrame, k13);

        final var k14 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame, oldEcefFrame, k14);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var k15 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k15);

        final var k16 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k16);

        final var k17 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition, k17);

        final var k18 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition, k18);

        final var k19 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, newEcefFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, k19);

        final var k20 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                k20);

        final var position = new InhomogeneousPoint3D(x, y, z);
        final var k21 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position, k21);

        final var k22 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position,
                k22);

        final var k23 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, velocity, oldVelocity, position,
                k23);

        final var k24 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, position, k24);

        final var k25 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position, k25);

        final var k26 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position, k26);

        final var newPositionK = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                newX, newY, newZ, newPositionK);

        // overloads receiving an explicit position, or both the new and the old frame, evaluate
        // gravity at the position at the start of the time interval
        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);

        // overloads receiving only the new frame evaluate gravity at the position at the end of the
        // time interval
        assertEquals(newPositionK, k9);
        assertEquals(newPositionK, k10);
        assertEquals(newPositionK, k11);
        assertEquals(newPositionK, k12);
        assertEquals(newPositionK, k19);
        assertEquals(newPositionK, k20);

        final var k = estimateKinematics(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        assertTrue(k1.equals(k, ABSOLUTE_ERROR));
    }

    /**
     * Checks that all static {@code estimateKinematicsAndReturnNew} overloads are consistent with each other and with
     * the in-test reference implementation for a randomly generated pair of frames.
     *
     * @param randomizer               randomizer used to generate the frames.
     * @param timeIntervalSeconds      time interval between frames expressed in seconds.
     * @param maxAngleVariationDegrees maximum absolute attitude variation per Euler angle between frames,
     *                                 expressed in degrees.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws WrongSizeException                            never happens.
     * @throws RankDeficientMatrixException                  never happens.
     * @throws DecomposerException                           never happens.
     */
    private static void checkEstimateKinematicsAndReturnNew(
            final UniformRandomizer randomizer, final double timeIntervalSeconds,
            final double maxAngleVariationDegrees) throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException, DecomposerException {

        final var oldNedFrame = createOldNedFrame(randomizer);
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame, randomizer, maxAngleVariationDegrees);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final var c = newEcefFrame.getCoordinateTransformation();
        final var oldC = oldEcefFrame.getCoordinateTransformation();

        final var vx = newEcefFrame.getVx();
        final var vy = newEcefFrame.getVy();
        final var vz = newEcefFrame.getVz();

        final var oldVx = oldEcefFrame.getVx();
        final var oldVy = oldEcefFrame.getVy();
        final var oldVz = oldEcefFrame.getVz();

        // position at the start of the time interval, where gravity must be evaluated
        final var x = oldEcefFrame.getX();
        final var y = oldEcefFrame.getY();
        final var z = oldEcefFrame.getZ();

        // position at the end of the time interval, used by overloads that only receive the new frame
        final var newX = newEcefFrame.getX();
        final var newY = newEcefFrame.getY();
        final var newZ = newEcefFrame.getZ();

        final var k1 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final var timeInterval = new Time(timeIntervalSeconds, TimeUnit.SECOND);
        final var k2 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z);

        final var velocity = new ECEFVelocity(vx, vy, vz);
        final var oldVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final var k3 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC, velocity,
                oldVelocity, x, y, z);

        final var k4 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, velocity,
                oldVelocity, x, y, z);

        final var ecefPosition = new ECEFPosition(x, y, z);
        final var k5 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition);

        final var k6 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, ecefPosition);

        final var k7 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC, velocity,
                oldVelocity, ecefPosition);

        final var k8 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, velocity,
                oldVelocity, ecefPosition);

        final var k9 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, newEcefFrame, oldC,
                oldVx, oldVy, oldVz);

        final var k10 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEcefFrame, oldC,
                oldVx, oldVy, oldVz);

        final var k11 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, newEcefFrame,
                oldC, oldVelocity);

        final var k12 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEcefFrame, oldC,
                oldVelocity);

        final var k13 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, newEcefFrame,
                oldEcefFrame);

        final var k14 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEcefFrame,
                oldEcefFrame);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var k15 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final var k16 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final var k17 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition);

        final var k18 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition);

        final var k19 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, newEcefFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final var k20 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEcefFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final var position = new InhomogeneousPoint3D(x, y, z);
        final var k21 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position);

        final var k22 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, position);

        final var k23 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC, velocity,
                oldVelocity, position);

        final var k24 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, velocity,
                oldVelocity, position);

        final var k25 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final var k26 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final var newPositionK = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeIntervalSeconds, c,
                oldC, vx, vy, vz, oldVx, oldVy, oldVz, newX, newY, newZ);

        // overloads receiving an explicit position, or both the new and the old frame, evaluate
        // gravity at the position at the start of the time interval
        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);

        // overloads receiving only the new frame evaluate gravity at the position at the end of the
        // time interval
        assertEquals(newPositionK, k9);
        assertEquals(newPositionK, k10);
        assertEquals(newPositionK, k11);
        assertEquals(newPositionK, k12);
        assertEquals(newPositionK, k19);
        assertEquals(newPositionK, k20);

        final var k = estimateKinematics(timeIntervalSeconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);

        assertTrue(k1.equals(k, ABSOLUTE_ERROR));
    }

    /**
     * Creates a randomizer with a fixed seed so that tests are deterministic.
     *
     * @return a seeded randomizer.
     */
    private static UniformRandomizer createRandomizer() {
        return new UniformRandomizer(new Random(SEED));
    }

    /**
     * Creates a random NED frame at the start of the time interval.
     *
     * @param randomizer randomizer used to generate the frame.
     * @return NED frame at the start of the time interval.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private static NEDFrame createOldNedFrame(final UniformRandomizer randomizer)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);
        return new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
    }

    /**
     * Creates a random NED frame at the end of the time interval by randomly modifying the frame at the start.
     *
     * @param oldFrame                 NED frame at the start of the time interval.
     * @param randomizer               randomizer used to generate the frame.
     * @param maxAngleVariationDegrees maximum absolute attitude variation per Euler angle, expressed in degrees.
     * @return NED frame at the end of the time interval.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     */
    private static NEDFrame createNewNedFrame(
            final NEDFrame oldFrame, final UniformRandomizer randomizer, final double maxAngleVariationDegrees)
            throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final var oldLatitude = oldFrame.getLatitude();
        final var oldLongitude = oldFrame.getLongitude();
        final var oldHeight = oldFrame.getHeight();

        final var oldVn = oldFrame.getVn();
        final var oldVe = oldFrame.getVe();
        final var oldVd = oldFrame.getVd();

        final var oldC = oldFrame.getCoordinateTransformation();

        final var oldRoll = oldC.getRollEulerAngle();
        final var oldPitch = oldC.getPitchEulerAngle();
        final var oldYaw = oldC.getYawEulerAngle();

        final var latitudeVariation = Math.toRadians(randomizer.nextDouble(MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final var longitudeVariation = Math.toRadians(randomizer.nextDouble(MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final var heightVariation = randomizer.nextDouble(MIN_HEIGHT_VARIATION, MAX_HEIGHT_VARIATION);

        final var vnVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);
        final var veVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);
        final var vdVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);

        final var rollVariation = Math.toRadians(randomizer.nextDouble(-maxAngleVariationDegrees,
                maxAngleVariationDegrees));
        final var pitchVariation = Math.toRadians(randomizer.nextDouble(-maxAngleVariationDegrees,
                maxAngleVariationDegrees));
        final var yawVariation = Math.toRadians(randomizer.nextDouble(-maxAngleVariationDegrees,
                maxAngleVariationDegrees));

        final var latitude = oldLatitude + latitudeVariation;
        final var longitude = oldLongitude + longitudeVariation;
        final var height = oldHeight + heightVariation;

        final var vn = oldVn + vnVariation;
        final var ve = oldVe + veVariation;
        final var vd = oldVd + vdVariation;

        final var roll = oldRoll + rollVariation;
        final var pitch = oldPitch + pitchVariation;
        final var yaw = oldYaw + yawVariation;

        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
    }

    private static BodyKinematics estimateKinematics(
            final double timeInterval, final CoordinateTransformation c, final CoordinateTransformation oldC,
            final double vx, final double vy, final double vz,
            final double oldVx, final double oldVy, final double oldVz, final double x, final double y, final double z)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {

        if (timeInterval > 0.0) {
            // From (2.145) determine the Earth rotation over the update interval
            final var omegaIe = Constants.EARTH_ROTATION_RATE;
            final var alphaIe = omegaIe * timeInterval;
            final var cEarth = CoordinateTransformation.eciToEcefMatrixFromAngle(alphaIe);
            final var cBe = c.getMatrix();
            final var oldCbe = oldC.getMatrix();
            final var cOldNew = cBe.transposeAndReturnNew().multiplyAndReturnNew(cEarth.multiplyAndReturnNew(oldCbe));

            // Calculate the approximate angular rate with respect an inertial frame
            final var alphaIbb = new Matrix(ROWS, 1);
            alphaIbb.setElementAtIndex(0,
                    0.5 * (cOldNew.getElementAt(1, 2) - cOldNew.getElementAt(2, 1)));
            alphaIbb.setElementAtIndex(1,
                    0.5 * (cOldNew.getElementAt(2, 0) - cOldNew.getElementAt(0, 2)));
            alphaIbb.setElementAtIndex(2,
                    0.5 * (cOldNew.getElementAt(0, 1) - cOldNew.getElementAt(1, 0)));

            // Calculate and apply the scaling factor
            final var temp = Math.acos(0.5 * (cOldNew.getElementAt(0, 0) +
                    cOldNew.getElementAt(1, 1) + cOldNew.getElementAt(2, 2) - 1.0));
            if (temp > SCALING_THRESHOLD) {
                // scaling is 1 if temp is less than this
                alphaIbb.multiplyByScalar(temp / Math.sin(temp));
            }

            // Calculate the angular rate
            final var omegaIbb = alphaIbb.multiplyByScalarAndReturnNew(1.0 / timeInterval);

            // Calculate the specific force resolved about ECEF-frame axes
            // Frame (5.36)
            final var vEbe = new Matrix(ROWS, 1);
            vEbe.setElementAtIndex(0, vx);
            vEbe.setElementAtIndex(1, vy);
            vEbe.setElementAtIndex(2, vz);

            final var oldVebe = new Matrix(ROWS, 1);
            oldVebe.setElementAtIndex(0, oldVx);
            oldVebe.setElementAtIndex(1, oldVy);
            oldVebe.setElementAtIndex(2, oldVz);

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(x, y, z);
            final var g = gravity.asMatrix();

            final var fIbe = (vEbe.subtractAndReturnNew(oldVebe).multiplyByScalarAndReturnNew(1.0 / timeInterval)
                    .subtractAndReturnNew(g).addAndReturnNew(Utils.skewMatrix(new double[]{0.0, 0.0, omegaIe})
                            .multiplyByScalarAndReturnNew(2.0).multiplyAndReturnNew(oldVebe)));

            // Calculate the average body-to-ECEF-frame coordinate transformation
            // matrix over the update interval using (5,84) and (5.85)
            final var magAlpha = Utils.normF(alphaIbb);
            final var skewAlphaIbb = Utils.skewMatrix(alphaIbb);
            final Matrix aveCbe;
            if (magAlpha > ALPHA_THRESHOLD) {
                aveCbe = oldCbe.multiplyAndReturnNew(Matrix.identity(ROWS, ROWS).addAndReturnNew(
                        skewAlphaIbb.multiplyByScalarAndReturnNew((1.0 - Math.cos(magAlpha)) / Math.pow(magAlpha, 2.0)))
                        .addAndReturnNew(skewAlphaIbb.multiplyAndReturnNew(skewAlphaIbb).multiplyByScalarAndReturnNew(
                                (1.0 - Math.sin(magAlpha) / magAlpha) / Math.pow(magAlpha, 2.0))))
                        .subtractAndReturnNew(Utils.skewMatrix(new double[]{0.0, 0.0, alphaIe})
                                .multiplyByScalarAndReturnNew(0.5).multiplyAndReturnNew(oldCbe));
            } else {
                aveCbe = oldCbe.subtractAndReturnNew(Utils.skewMatrix(new double[]{0.0, 0.0, alphaIe})
                        .multiplyByScalarAndReturnNew(0.5).multiplyAndReturnNew(oldCbe));
            }

            // Transform specific force to body-frame resolving axes using (5.81)
            final var fIbb = Utils.inverse(aveCbe).multiplyAndReturnNew(fIbe);

            final var specificForceX = fIbb.getElementAtIndex(0);
            final var specificForceY = fIbb.getElementAtIndex(1);
            final var specificForceZ = fIbb.getElementAtIndex(2);

            final var angularRateX = omegaIbb.getElementAtIndex(0);
            final var angularRateY = omegaIbb.getElementAtIndex(1);
            final var angularRateZ = omegaIbb.getElementAtIndex(2);

            // save result data
            return new BodyKinematics(specificForceX, specificForceY, specificForceZ,
                    angularRateX, angularRateY, angularRateZ);
        } else {
            // If a time interval is zero, set angular rate and specific force to zero
            return new BodyKinematics();
        }
    }

    /**
     * Creates the ECEF frame at the start of the interval used by the navigator round trip tests.
     *
     * @return ECEF frame at the start of the interval.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private static ECEFFrame createRoundTripOldFrame() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final var q = new Quaternion(Math.toRadians(ROUND_TRIP_ROLL_DEGREES),
                Math.toRadians(ROUND_TRIP_PITCH_DEGREES), Math.toRadians(ROUND_TRIP_YAW_DEGREES));
        final var c = new CoordinateTransformation(q.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT, ROUND_TRIP_VN, ROUND_TRIP_VE, ROUND_TRIP_VD, c);
        return NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
    }

    /**
     * Creates the body kinematics used by the navigator round trip tests.
     *
     * @return body kinematics applied during the interval.
     */
    private static BodyKinematics createRoundTripKinematics() {
        return new BodyKinematics(ROUND_TRIP_FX, ROUND_TRIP_FY, ROUND_TRIP_FZ,
                ROUND_TRIP_ANGULAR_RATE_X, ROUND_TRIP_ANGULAR_RATE_Y, ROUND_TRIP_ANGULAR_RATE_Z);
    }

    /**
     * Computes the Euclidean distance between the specific forces of two body kinematics.
     *
     * @param k1 first body kinematics.
     * @param k2 second body kinematics.
     * @return distance between specific forces expressed in meters per squared second.
     */
    private static double specificForceDistance(final BodyKinematics k1, final BodyKinematics k2) {
        final var dx = k1.getFx() - k2.getFx();
        final var dy = k1.getFy() - k2.getFy();
        final var dz = k1.getFz() - k2.getFz();
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * Computes the Euclidean distance between the angular rates of two body kinematics.
     *
     * @param k1 first body kinematics.
     * @param k2 second body kinematics.
     * @return distance between angular rates expressed in radians per second.
     */
    private static double angularRateDistance(final BodyKinematics k1, final BodyKinematics k2) {
        final var dx = k1.getAngularRateX() - k2.getAngularRateX();
        final var dy = k1.getAngularRateY() - k2.getAngularRateY();
        final var dz = k1.getAngularRateZ() - k2.getAngularRateZ();
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
}
