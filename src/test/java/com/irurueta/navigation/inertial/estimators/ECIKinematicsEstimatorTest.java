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
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.navigators.ECIInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class ECIKinematicsEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Large time interval, expressed in seconds (s), used by a second pass of the estimation tests along
     * with {@link #LARGE_MAX_ANGLE_VARIATION_DEGREES} so that second order terms (attitude change during the
     * time interval and gravitation change between old and new positions) are observable.
     */
    private static final double LARGE_TIME_INTERVAL_SECONDS = 1.0;

    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    /**
     * Maximum absolute variation, expressed in degrees, of each Euler angle between old and new frames in the
     * small pass of the estimation tests.
     */
    private static final double MAX_ANGLE_VARIATION_DEGREES = 5.0;

    /**
     * Maximum absolute variation, expressed in degrees, of each Euler angle between old and new frames in the
     * large pass of the estimation tests.
     */
    private static final double LARGE_MAX_ANGLE_VARIATION_DEGREES = 45.0;

    private static final double MIN_POSITION_VARIATION_DEGREES = -1e-4;
    private static final double MAX_POSITION_VARIATION_DEGREES = 1e-4;

    private static final double MIN_HEIGHT_VARIATION = -0.5;
    private static final double MAX_HEIGHT_VARIATION = 0.5;

    private static final double MIN_VELOCITY_VARIATION = -0.1;
    private static final double MAX_VELOCITY_VARIATION = 0.1;

    private static final double SCALING_THRESHOLD = 2e-5;
    private static final double ALPHA_THRESHOLD = 1e-8;

    /**
     * Maximum allowed difference between the estimator and the in-test reference implementation
     * (a direct transcription of Groves' {@code Kinematics_ECI.m} using matrix operations). Both perform the
     * same arithmetic and only differ in the order of a few floating point operations, hence only rounding
     * errors remain: ECI velocities are of the order of 1e2 m/s, so their rounding error (about 1e-14 m/s)
     * divided by the shortest time interval (0.02 s) is of the order of 1e-12 m/s^2. This value passes for
     * both passes (small and large) of the {@link #SEEDED_RUNS} seeded runs of the estimation tests, which
     * were observed to keep passing when this value is reduced down to 1e-13, so no larger value is needed.
     */
    private static final double ABSOLUTE_ERROR = 1e-9;

    /**
     * Base seed of the randomizers used by the estimation tests, so that they are deterministic.
     */
    private static final long SEED = 36L;

    /**
     * Number of seeded runs executed by the estimation tests. Run {@code i} uses seed {@code SEED + i}.
     */
    private static final int SEEDED_RUNS = 1000;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    /**
     * North velocity, expressed in meters per second (m/s), of the fixed frame used by the round trip tests.
     */
    private static final double ROUND_TRIP_VN = 30.0;

    /**
     * East velocity, expressed in meters per second (m/s), of the fixed frame used by the round trip tests.
     */
    private static final double ROUND_TRIP_VE = 20.0;

    /**
     * Down velocity, expressed in meters per second (m/s), of the fixed frame used by the round trip tests.
     */
    private static final double ROUND_TRIP_VD = -1.0;

    /**
     * Roll angle, expressed in degrees, of the fixed frame used by the round trip tests.
     */
    private static final double ROUND_TRIP_ROLL_DEGREES = 10.0;

    /**
     * Pitch angle, expressed in degrees, of the fixed frame used by the round trip tests.
     */
    private static final double ROUND_TRIP_PITCH_DEGREES = -20.0;

    /**
     * Yaw angle, expressed in degrees, of the fixed frame used by the round trip tests.
     */
    private static final double ROUND_TRIP_YAW_DEGREES = 30.0;

    /**
     * X coordinate of specific force, expressed in meters per squared second (m/s^2), used by the round trip
     * tests.
     */
    private static final double ROUND_TRIP_FX = 1.0;

    /**
     * Y coordinate of specific force, expressed in meters per squared second (m/s^2), used by the round trip
     * tests.
     */
    private static final double ROUND_TRIP_FY = 2.0;

    /**
     * Z coordinate of specific force, expressed in meters per squared second (m/s^2), used by the round trip
     * tests.
     */
    private static final double ROUND_TRIP_FZ = -9.8;

    /**
     * X coordinate of angular rate, expressed in radians per second (rad/s), used by the round trip tests.
     */
    private static final double ROUND_TRIP_ANGULAR_RATE_X = 0.5;

    /**
     * Y coordinate of angular rate, expressed in radians per second (rad/s), used by the round trip tests.
     */
    private static final double ROUND_TRIP_ANGULAR_RATE_Y = -0.3;

    /**
     * Z coordinate of angular rate, expressed in radians per second (rad/s), used by the round trip tests.
     */
    private static final double ROUND_TRIP_ANGULAR_RATE_Z = 1.0;

    /**
     * Short time interval, expressed in seconds (s), used by the round trip tests.
     */
    private static final double ROUND_TRIP_SHORT_TIME_INTERVAL_SECONDS = 0.1;

    /**
     * Long time interval, expressed in seconds (s), used by the round trip tests. With the angular rate used
     * by the round trip tests the body rotates more than one radian during this interval, so that second
     * order attitude terms are clearly observable.
     */
    private static final double ROUND_TRIP_LONG_TIME_INTERVAL_SECONDS = 1.0;

    /**
     * Maximum allowed difference between the kinematics fed to {@link ECIInertialNavigator} and the
     * kinematics recovered by {@link ECIKinematicsEstimator} from the navigated frame. Estimator and
     * navigator are exact inverses of each other when gravitation is evaluated at the position at the start
     * of the time interval, hence only rounding errors remain: velocities are of the order of 1e2 m/s
     * (they include Earth rotation), so that their rounding error (about 1e-14 m/s) divided by the shortest
     * time interval (0.1 s) stays several orders of magnitude below this value (the round trip was observed
     * to keep passing when this value is reduced down to 1e-12). Any deviation from the reference algorithm
     * is orders of magnitude larger (1e-6 m/s^2 or more).
     */
    private static final double ROUND_TRIP_ABSOLUTE_ERROR = 1e-9;

    /**
     * Minimum specific force difference, expressed in meters per squared second (m/s^2), that must exist
     * between kinematics estimated with gravitation evaluated at the old position and at the new position
     * for the long round trip time interval. In that interval the body moves several hundred meters with
     * respect to the ECI frame (because of Earth rotation), and gravitation gradient is about 1.5e-6 s^-2,
     * so the actual difference is of the order of 1e-4 m/s^2, well above this value and well above
     * {@link #ROUND_TRIP_ABSOLUTE_ERROR}.
     */
    private static final double MIN_POSITION_SPECIFIC_FORCE_DIFFERENCE = 1e-6;

    /**
     * Checks that all {@link ECIKinematicsEstimator#estimate} overloads are consistent:
     * overloads receiving an explicit position or the previous frame evaluate gravitation at the provided
     * or previous position, and overloads receiving only the current frame evaluate gravitation at the
     * current position.
     * Each seeded run executes a small pass (short time interval, small attitude variation) and a large pass
     * (long time interval, large attitude variation).
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    @Test
    void testEstimate() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        for (var i = 0; i < SEEDED_RUNS; i++) {
            final var randomizer = new UniformRandomizer(new Random(SEED + i));
            checkEstimate(randomizer, TIME_INTERVAL_SECONDS, MAX_ANGLE_VARIATION_DEGREES);
            checkEstimate(randomizer, LARGE_TIME_INTERVAL_SECONDS, LARGE_MAX_ANGLE_VARIATION_DEGREES);
        }
    }

    /**
     * Checks that all {@link ECIKinematicsEstimator#estimateAndReturnNew} overloads are consistent:
     * overloads receiving an explicit position or the previous frame evaluate gravitation at the provided
     * or previous position, and overloads receiving only the current frame evaluate gravitation at the
     * current position.
     * Each seeded run executes a small pass (short time interval, small attitude variation) and a large pass
     * (long time interval, large attitude variation).
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    @Test
    void testEstimateAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        for (var i = 0; i < SEEDED_RUNS; i++) {
            final var randomizer = new UniformRandomizer(new Random(SEED + i));
            checkEstimateAndReturnNew(randomizer, TIME_INTERVAL_SECONDS, MAX_ANGLE_VARIATION_DEGREES);
            checkEstimateAndReturnNew(randomizer, LARGE_TIME_INTERVAL_SECONDS, LARGE_MAX_ANGLE_VARIATION_DEGREES);
        }
    }

    /**
     * Checks that all {@link ECIKinematicsEstimator#estimateKinematics} overloads are consistent:
     * overloads receiving an explicit position or the previous frame evaluate gravitation at the provided
     * or previous position, and overloads receiving only the current frame evaluate gravitation at the
     * current position.
     * Results are also compared against the in-test reference implementation.
     * Each seeded run executes a small pass (short time interval, small attitude variation) and a large pass
     * (long time interval, large attitude variation).
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws WrongSizeException                            never happens.
     * @throws RankDeficientMatrixException                  never happens.
     * @throws DecomposerException                           never happens.
     */
    @Test
    void testEstimateKinematics() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException,
            WrongSizeException, RankDeficientMatrixException, DecomposerException {

        for (var i = 0; i < SEEDED_RUNS; i++) {
            final var randomizer = new UniformRandomizer(new Random(SEED + i));
            checkEstimateKinematics(randomizer, TIME_INTERVAL_SECONDS, MAX_ANGLE_VARIATION_DEGREES);
            checkEstimateKinematics(randomizer, LARGE_TIME_INTERVAL_SECONDS, LARGE_MAX_ANGLE_VARIATION_DEGREES);
        }
    }

    /**
     * Checks that all {@link ECIKinematicsEstimator#estimateKinematicsAndReturnNew} overloads are consistent:
     * overloads receiving an explicit position or the previous frame evaluate gravitation at the provided
     * or previous position, and overloads receiving only the current frame evaluate gravitation at the
     * current position.
     * Results are also compared against the in-test reference implementation.
     * Each seeded run executes a small pass (short time interval, small attitude variation) and a large pass
     * (long time interval, large attitude variation).
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws WrongSizeException                            never happens.
     * @throws RankDeficientMatrixException                  never happens.
     * @throws DecomposerException                           never happens.
     */
    @Test
    void testEstimateKinematicsAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException, DecomposerException {

        for (var i = 0; i < SEEDED_RUNS; i++) {
            final var randomizer = new UniformRandomizer(new Random(SEED + i));
            checkEstimateKinematicsAndReturnNew(randomizer, TIME_INTERVAL_SECONDS, MAX_ANGLE_VARIATION_DEGREES);
            checkEstimateKinematicsAndReturnNew(randomizer, LARGE_TIME_INTERVAL_SECONDS,
                    LARGE_MAX_ANGLE_VARIATION_DEGREES);
        }
    }

    @Test
    void testEstimateKinematicsWhenNegativeIntervalThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame();
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                oldEcefFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame);

        final var c = newEciFrame.getCoordinateTransformation();
        final var oldC = oldEciFrame.getCoordinateTransformation();

        final var vx = newEciFrame.getVx();
        final var vy = newEciFrame.getVy();
        final var vz = newEciFrame.getVz();

        final var oldVx = oldEciFrame.getVx();
        final var oldVy = oldEciFrame.getVy();
        final var oldVz = oldEciFrame.getVz();

        final var x = newEciFrame.getX();
        final var y = newEciFrame.getY();
        final var z = newEciFrame.getZ();

        assertThrows(IllegalArgumentException.class, () -> ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                -1.0, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z));
    }

    @Test
    void testEstimateKinematicsWhenInvalidCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame();
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                oldEcefFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame);

        final var c = newEciFrame.getCoordinateTransformation();
        final var oldC = oldEciFrame.getCoordinateTransformation();

        final var vx = newEciFrame.getVx();
        final var vy = newEciFrame.getVy();
        final var vz = newEciFrame.getVz();

        final var oldVx = oldEciFrame.getVx();
        final var oldVy = oldEciFrame.getVy();
        final var oldVz = oldEciFrame.getVz();

        final var x = newEciFrame.getX();
        final var y = newEciFrame.getY();
        final var z = newEciFrame.getZ();

        c.setDestinationType(FrameType.BODY_FRAME);
        assertThrows(IllegalArgumentException.class, () -> ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z));
    }

    @Test
    void testEstimateKinematicsWhenInvalidOldCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame();
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                oldEcefFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame);

        final var c = newEciFrame.getCoordinateTransformation();
        final var oldC = oldEciFrame.getCoordinateTransformation();

        final var vx = newEciFrame.getVx();
        final var vy = newEciFrame.getVy();
        final var vz = newEciFrame.getVz();

        final var oldVx = oldEciFrame.getVx();
        final var oldVy = oldEciFrame.getVy();
        final var oldVz = oldEciFrame.getVz();

        final var x = newEciFrame.getX();
        final var y = newEciFrame.getY();
        final var z = newEciFrame.getZ();

        oldC.setDestinationType(FrameType.BODY_FRAME);
        assertThrows(IllegalArgumentException.class, () -> ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z));
    }

    @Test
    void testEstimateKinematicsWhenZeroTimeIntervalReturnsZeroValues() 
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        final var oldNedFrame = createOldNedFrame();
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldEcefFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, 
                newEcefFrame);

        final var c = newEciFrame.getCoordinateTransformation();
        final var oldC = oldEciFrame.getCoordinateTransformation();

        final var vx = newEciFrame.getVx();
        final var vy = newEciFrame.getVy();
        final var vz = newEciFrame.getVz();

        final var oldVx = oldEciFrame.getVx();
        final var oldVy = oldEciFrame.getVy();
        final var oldVz = oldEciFrame.getVz();

        final var x = newEciFrame.getX();
        final var y = newEciFrame.getY();
        final var z = newEciFrame.getZ();

        final var k = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(0.0, c, oldC, vx, vy, vz, 
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

    @Test
    void testCompareKinematics() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {
        final var oldNedFrame = createOldNedFrame();
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldEcefFrame);

        final var newNedFrame = createNewNedFrame(oldNedFrame);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, 
                newEcefFrame);

        final var nedK = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newNedFrame,
                oldNedFrame);
        final var ecefK = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame,
                oldEcefFrame);
        final var eciK = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame,
                oldEciFrame);

        final var nedSpecificForceNorm = nedK.getSpecificForceNorm();
        final var ecefSpecificForceNorm = ecefK.getSpecificForceNorm();
        final var eciSpecificForceNorm = eciK.getSpecificForceNorm();

        final var nedAngularRateNorm = nedK.getAngularRateNorm();
        final var ecefAngularRateNorm = ecefK.getAngularRateNorm();
        final var eciAngularRateNorm = eciK.getAngularRateNorm();

        assertEquals(ecefSpecificForceNorm, nedSpecificForceNorm, LARGE_ABSOLUTE_ERROR);
        assertEquals(eciSpecificForceNorm, nedSpecificForceNorm, LARGE_ABSOLUTE_ERROR);

        assertEquals(ecefAngularRateNorm, nedAngularRateNorm, LARGE_ABSOLUTE_ERROR);
        assertEquals(eciAngularRateNorm, nedAngularRateNorm, LARGE_ABSOLUTE_ERROR);

        assertEquals(ecefK.getFx(), nedK.getFx(), LARGE_ABSOLUTE_ERROR);
        assertEquals(ecefK.getFy(), nedK.getFy(), LARGE_ABSOLUTE_ERROR);
        assertEquals(ecefK.getFz(), nedK.getFz(), LARGE_ABSOLUTE_ERROR);
        assertEquals(ecefK.getAngularRateX(), nedK.getAngularRateX(), LARGE_ABSOLUTE_ERROR);
        assertEquals(ecefK.getAngularRateY(), nedK.getAngularRateY(), LARGE_ABSOLUTE_ERROR);
        assertEquals(ecefK.getAngularRateZ(), nedK.getAngularRateZ(), LARGE_ABSOLUTE_ERROR);

        assertEquals(eciK.getFx(), nedK.getFx(), LARGE_ABSOLUTE_ERROR);
        assertEquals(eciK.getFy(), nedK.getFy(), LARGE_ABSOLUTE_ERROR);
        assertEquals(eciK.getFz(), nedK.getFz(), LARGE_ABSOLUTE_ERROR);
        assertEquals(eciK.getAngularRateX(), nedK.getAngularRateX(), LARGE_ABSOLUTE_ERROR);
        assertEquals(eciK.getAngularRateY(), nedK.getAngularRateY(), LARGE_ABSOLUTE_ERROR);
        assertEquals(eciK.getAngularRateZ(), nedK.getAngularRateZ(), LARGE_ABSOLUTE_ERROR);

        assertTrue(ecefK.equals(nedK, LARGE_ABSOLUTE_ERROR));
        assertTrue(eciK.equals(nedK, LARGE_ABSOLUTE_ERROR));
    }

    /**
     * Checks that kinematics fed to {@link ECIInertialNavigator} on a rotating body are recovered by the
     * estimator from the navigated frame when gravitation is evaluated at the position at the start of the
     * time interval.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws InertialNavigatorException                    never happens.
     */
    @Test
    void testNavigatorRoundTripWithRotation() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, InertialNavigatorException {

        final var timeIntervals = new double[]{
                ROUND_TRIP_SHORT_TIME_INTERVAL_SECONDS, ROUND_TRIP_LONG_TIME_INTERVAL_SECONDS};
        for (final var timeInterval : timeIntervals) {
            final var oldFrame = createRoundTripOldEciFrame(timeInterval);
            final var kinematics = createRoundTripKinematics();

            final var oldC = oldFrame.getCoordinateTransformation();
            final var newFrame = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval,
                    oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(), oldC,
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), kinematics);

            final var result = new BodyKinematics();
            ECIKinematicsEstimator.estimateKinematics(timeInterval, newFrame.getCoordinateTransformation(), oldC,
                    newFrame.getVx(), newFrame.getVy(), newFrame.getVz(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(),
                    oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(), result);

            assertTrue(result.equals(kinematics, ROUND_TRIP_ABSOLUTE_ERROR),
                    "time interval: " + timeInterval + ", expected: " + describe(kinematics)
                            + ", estimated: " + describe(result));
        }
    }

    /**
     * Checks that every overload receiving both the current and the previous frame evaluates gravitation at
     * the previous position (the position at the start of the time interval), so that it recovers the
     * kinematics fed to {@link ECIInertialNavigator}.
     *
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws InertialNavigatorException                    never happens.
     */
    @Test
    void testFrameOverloadsUseOldPosition() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, InertialNavigatorException {

        final var seconds = ROUND_TRIP_LONG_TIME_INTERVAL_SECONDS;
        final var timeInterval = new Time(seconds, TimeUnit.SECOND);

        final var oldFrame = createRoundTripOldEciFrame(seconds);
        final var kinematics = createRoundTripKinematics();
        final var newFrame = ECIInertialNavigator.navigateECIAndReturnNew(seconds, oldFrame, kinematics);

        final var c = newFrame.getCoordinateTransformation();
        final var oldC = oldFrame.getCoordinateTransformation();

        // reference results obtained with an explicit position
        final var oldPositionResult = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds, c, oldC,
                newFrame.getVx(), newFrame.getVy(), newFrame.getVz(),
                oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(),
                oldFrame.getX(), oldFrame.getY(), oldFrame.getZ());
        final var newPositionResult = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds, c, oldC,
                newFrame.getVx(), newFrame.getVy(), newFrame.getVz(),
                oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(),
                newFrame.getX(), newFrame.getY(), newFrame.getZ());

        // static overloads
        final var k1 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds, newFrame, oldFrame, k1);

        final var k2 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval, newFrame, oldFrame, k2);

        final var k3 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds, newFrame, oldFrame);

        final var k4 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newFrame, oldFrame);

        // instance overloads
        final var estimator = new ECIKinematicsEstimator();

        final var k5 = new BodyKinematics();
        estimator.estimate(seconds, newFrame, oldFrame, k5);

        final var k6 = new BodyKinematics();
        estimator.estimate(timeInterval, newFrame, oldFrame, k6);

        final var k7 = estimator.estimateAndReturnNew(seconds, newFrame, oldFrame);

        final var k8 = estimator.estimateAndReturnNew(timeInterval, newFrame, oldFrame);

        final var results = new BodyKinematics[]{k1, k2, k3, k4, k5, k6, k7, k8};
        for (var i = 0; i < results.length; i++) {
            final var result = results[i];
            final var message = "overload: k" + (i + 1) + ", estimated: " + describe(result);

            // gravitation is evaluated at the position at the start of the time interval
            assertEquals(oldPositionResult, result, message);

            // and consequently the kinematics fed to the navigator are recovered
            assertTrue(result.equals(kinematics, ROUND_TRIP_ABSOLUTE_ERROR), message);

            // while evaluating gravitation at the position at the end of the time interval is
            // observably different
            assertTrue(specificForceDifference(result, newPositionResult) > MIN_POSITION_SPECIFIC_FORCE_DIFFERENCE,
                    message);
        }
    }

    /**
     * Executes one pass of {@link #testEstimate()}.
     *
     * @param randomizer               seeded randomizer used to create old and new frames.
     * @param seconds                  time interval between old and new frames expressed in seconds (s).
     * @param maxAngleVariationDegrees maximum absolute variation, expressed in degrees, of each Euler angle
     *                                 between old and new frames.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private static void checkEstimate(
            final UniformRandomizer randomizer, final double seconds, final double maxAngleVariationDegrees)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame(randomizer);
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(seconds, oldEcefFrame);

        final var newNedFrame = createNewNedFrame(randomizer, oldNedFrame, maxAngleVariationDegrees);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(seconds, newEcefFrame);

        final var c = newEciFrame.getCoordinateTransformation();
        final var oldC = oldEciFrame.getCoordinateTransformation();

        final var vx = newEciFrame.getVx();
        final var vy = newEciFrame.getVy();
        final var vz = newEciFrame.getVz();

        final var oldVx = oldEciFrame.getVx();
        final var oldVy = oldEciFrame.getVy();
        final var oldVz = oldEciFrame.getVz();

        // position at the end of the time interval
        final var x = newEciFrame.getX();
        final var y = newEciFrame.getY();
        final var z = newEciFrame.getZ();

        // position at the start of the time interval
        final var oldX = oldEciFrame.getX();
        final var oldY = oldEciFrame.getY();
        final var oldZ = oldEciFrame.getZ();
        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);

        final var timeInterval = new Time(seconds, TimeUnit.SECOND);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var estimator = new ECIKinematicsEstimator();

        // k1 is the reference result for the old position (start of the time interval), and
        // kNewPosition is the reference result for the new position (end of the time interval)
        final var k1 = new BodyKinematics();
        estimator.estimate(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldX, oldY, oldZ, k1);

        final var k2 = new BodyKinematics();
        estimator.estimate(timeInterval,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldX, oldY, oldZ, k2);

        final var k3 = new BodyKinematics();
        estimator.estimate(seconds,
                newEciFrame, oldC, oldVx, oldVy, oldVz, k3);

        final var k4 = new BodyKinematics();
        estimator.estimate(timeInterval,
                newEciFrame, oldC, oldVx, oldVy, oldVz, k4);

        final var k5 = new BodyKinematics();
        estimator.estimate(seconds,
                newEciFrame, oldEciFrame, k5);

        final var k6 = new BodyKinematics();
        estimator.estimate(timeInterval,
                newEciFrame, oldEciFrame, k6);

        final var k7 = new BodyKinematics();
        estimator.estimate(seconds,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldX, oldY, oldZ, k7);

        final var k8 = new BodyKinematics();
        estimator.estimate(timeInterval,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldX, oldY, oldZ, k8);

        final var k9 = new BodyKinematics();
        estimator.estimate(seconds,
                newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k9);

        final var k10 = new BodyKinematics();
        estimator.estimate(timeInterval,
                newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k10);

        final var k11 = new BodyKinematics();
        estimator.estimate(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldPosition, k11);

        final var k12 = new BodyKinematics();
        estimator.estimate(timeInterval,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldPosition, k12);

        final var k13 = new BodyKinematics();
        estimator.estimate(seconds,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldPosition, k13);

        final var k14 = new BodyKinematics();
        estimator.estimate(timeInterval,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldPosition, k14);

        final var kNewPosition = new BodyKinematics();
        estimator.estimate(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, kNewPosition);

        // both reference results are different because the body has moved
        assertNotEquals(k1, kNewPosition);

        // overloads receiving an explicit (old) position
        assertEquals(k1, k2);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);

        // overloads receiving the old frame use the old position
        assertEquals(k1, k5);
        assertEquals(k1, k6);

        // overloads receiving only the new frame use the new position
        assertEquals(kNewPosition, k3);
        assertEquals(kNewPosition, k4);
        assertEquals(kNewPosition, k9);
        assertEquals(kNewPosition, k10);
    }

    /**
     * Executes one pass of {@link #testEstimateAndReturnNew()}.
     *
     * @param randomizer               seeded randomizer used to create old and new frames.
     * @param seconds                  time interval between old and new frames expressed in seconds (s).
     * @param maxAngleVariationDegrees maximum absolute variation, expressed in degrees, of each Euler angle
     *                                 between old and new frames.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private static void checkEstimateAndReturnNew(
            final UniformRandomizer randomizer, final double seconds, final double maxAngleVariationDegrees)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldNedFrame = createOldNedFrame(randomizer);
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(seconds, oldEcefFrame);

        final var newNedFrame = createNewNedFrame(randomizer, oldNedFrame, maxAngleVariationDegrees);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(seconds, newEcefFrame);

        final var c = newEciFrame.getCoordinateTransformation();
        final var oldC = oldEciFrame.getCoordinateTransformation();

        final var vx = newEciFrame.getVx();
        final var vy = newEciFrame.getVy();
        final var vz = newEciFrame.getVz();

        final var oldVx = oldEciFrame.getVx();
        final var oldVy = oldEciFrame.getVy();
        final var oldVz = oldEciFrame.getVz();

        // position at the end of the time interval
        final var x = newEciFrame.getX();
        final var y = newEciFrame.getY();
        final var z = newEciFrame.getZ();

        // position at the start of the time interval
        final var oldX = oldEciFrame.getX();
        final var oldY = oldEciFrame.getY();
        final var oldZ = oldEciFrame.getZ();
        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);

        final var timeInterval = new Time(seconds, TimeUnit.SECOND);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var estimator = new ECIKinematicsEstimator();

        // k1 is the reference result for the old position (start of the time interval), and
        // kNewPosition is the reference result for the new position (end of the time interval)
        final var k1 = estimator.estimateAndReturnNew(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldX, oldY, oldZ);

        final var k2 = estimator.estimateAndReturnNew(timeInterval,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldX, oldY, oldZ);

        final var k3 = estimator.estimateAndReturnNew(seconds,
                newEciFrame, oldC, oldVx, oldVy, oldVz);

        final var k4 = estimator.estimateAndReturnNew(timeInterval,
                newEciFrame, oldC, oldVx, oldVy, oldVz);

        final var k5 = estimator.estimateAndReturnNew(seconds,
                newEciFrame, oldEciFrame);

        final var k6 = estimator.estimateAndReturnNew(timeInterval,
                newEciFrame, oldEciFrame);

        final var k7 = estimator.estimateAndReturnNew(seconds,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldX, oldY, oldZ);

        final var k8 = estimator.estimateAndReturnNew(timeInterval,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldX, oldY, oldZ);

        final var k9 = estimator.estimateAndReturnNew(seconds,
                newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final var k10 = estimator.estimateAndReturnNew(timeInterval,
                newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final var k11 = estimator.estimateAndReturnNew(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldPosition);

        final var k12 = estimator.estimateAndReturnNew(timeInterval,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldPosition);

        final var k13 = estimator.estimateAndReturnNew(seconds,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldPosition);

        final var k14 = estimator.estimateAndReturnNew(timeInterval,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldPosition);

        final var kNewPosition = estimator.estimateAndReturnNew(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        // both reference results are different because the body has moved
        assertNotEquals(k1, kNewPosition);

        // overloads receiving an explicit (old) position
        assertEquals(k1, k2);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);

        // overloads receiving the old frame use the old position
        assertEquals(k1, k5);
        assertEquals(k1, k6);

        // overloads receiving only the new frame use the new position
        assertEquals(kNewPosition, k3);
        assertEquals(kNewPosition, k4);
        assertEquals(kNewPosition, k9);
        assertEquals(kNewPosition, k10);
    }

    /**
     * Executes one pass of {@link #testEstimateKinematics()}.
     *
     * @param randomizer               seeded randomizer used to create old and new frames.
     * @param seconds                  time interval between old and new frames expressed in seconds (s).
     * @param maxAngleVariationDegrees maximum absolute variation, expressed in degrees, of each Euler angle
     *                                 between old and new frames.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws WrongSizeException                            never happens.
     * @throws RankDeficientMatrixException                  never happens.
     * @throws DecomposerException                           never happens.
     */
    private static void checkEstimateKinematics(
            final UniformRandomizer randomizer, final double seconds, final double maxAngleVariationDegrees)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException,
            WrongSizeException, RankDeficientMatrixException, DecomposerException {

        final var oldNedFrame = createOldNedFrame(randomizer);
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(seconds, oldEcefFrame);

        final var newNedFrame = createNewNedFrame(randomizer, oldNedFrame, maxAngleVariationDegrees);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(seconds, newEcefFrame);

        final var c = newEciFrame.getCoordinateTransformation();
        final var oldC = oldEciFrame.getCoordinateTransformation();

        final var vx = newEciFrame.getVx();
        final var vy = newEciFrame.getVy();
        final var vz = newEciFrame.getVz();

        final var oldVx = oldEciFrame.getVx();
        final var oldVy = oldEciFrame.getVy();
        final var oldVz = oldEciFrame.getVz();

        // position at the end of the time interval
        final var x = newEciFrame.getX();
        final var y = newEciFrame.getY();
        final var z = newEciFrame.getZ();

        // position at the start of the time interval
        final var oldX = oldEciFrame.getX();
        final var oldY = oldEciFrame.getY();
        final var oldZ = oldEciFrame.getZ();
        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);

        final var timeInterval = new Time(seconds, TimeUnit.SECOND);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        // k1 is the reference result for the old position (start of the time interval), and
        // kNewPosition is the reference result for the new position (end of the time interval)
        final var k1 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldX, oldY, oldZ, k1);

        final var k2 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldX, oldY, oldZ, k2);

        final var k3 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds,
                newEciFrame, oldC, oldVx, oldVy, oldVz, k3);

        final var k4 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval,
                newEciFrame, oldC, oldVx, oldVy, oldVz, k4);

        final var k5 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds,
                newEciFrame, oldEciFrame, k5);

        final var k6 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval,
                newEciFrame, oldEciFrame, k6);

        final var k7 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldX, oldY, oldZ, k7);

        final var k8 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldX, oldY, oldZ, k8);

        final var k9 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds,
                newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k9);

        final var k10 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval,
                newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k10);

        final var k11 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldPosition, k11);

        final var k12 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldPosition, k12);

        final var k13 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldPosition, k13);

        final var k14 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldPosition, k14);

        final var kNewPosition = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, kNewPosition);

        // both reference results are different because the body has moved
        assertNotEquals(k1, kNewPosition);

        // overloads receiving an explicit (old) position
        assertEquals(k1, k2);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);

        // overloads receiving the old frame use the old position
        assertEquals(k1, k5);
        assertEquals(k1, k6);

        // overloads receiving only the new frame use the new position
        assertEquals(kNewPosition, k3);
        assertEquals(kNewPosition, k4);
        assertEquals(kNewPosition, k9);
        assertEquals(kNewPosition, k10);

        // compare against the in-test reference implementation
        final var expected = estimateKinematics(seconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                oldX, oldY, oldZ);
        assertTrue(k1.equals(expected, ABSOLUTE_ERROR));

        final var expectedNewPosition = estimateKinematics(seconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);
        assertTrue(kNewPosition.equals(expectedNewPosition, ABSOLUTE_ERROR));
    }

    /**
     * Executes one pass of {@link #testEstimateKinematicsAndReturnNew()}.
     *
     * @param randomizer               seeded randomizer used to create old and new frames.
     * @param seconds                  time interval between old and new frames expressed in seconds (s).
     * @param maxAngleVariationDegrees maximum absolute variation, expressed in degrees, of each Euler angle
     *                                 between old and new frames.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws WrongSizeException                            never happens.
     * @throws RankDeficientMatrixException                  never happens.
     * @throws DecomposerException                           never happens.
     */
    private static void checkEstimateKinematicsAndReturnNew(
            final UniformRandomizer randomizer, final double seconds, final double maxAngleVariationDegrees)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException,
            WrongSizeException, RankDeficientMatrixException, DecomposerException {

        final var oldNedFrame = createOldNedFrame(randomizer);
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(seconds, oldEcefFrame);

        final var newNedFrame = createNewNedFrame(randomizer, oldNedFrame, maxAngleVariationDegrees);
        final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(seconds, newEcefFrame);

        final var c = newEciFrame.getCoordinateTransformation();
        final var oldC = oldEciFrame.getCoordinateTransformation();

        final var vx = newEciFrame.getVx();
        final var vy = newEciFrame.getVy();
        final var vz = newEciFrame.getVz();

        final var oldVx = oldEciFrame.getVx();
        final var oldVy = oldEciFrame.getVy();
        final var oldVz = oldEciFrame.getVz();

        // position at the end of the time interval
        final var x = newEciFrame.getX();
        final var y = newEciFrame.getY();
        final var z = newEciFrame.getZ();

        // position at the start of the time interval
        final var oldX = oldEciFrame.getX();
        final var oldY = oldEciFrame.getY();
        final var oldZ = oldEciFrame.getZ();
        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);

        final var timeInterval = new Time(seconds, TimeUnit.SECOND);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        // k1 is the reference result for the old position (start of the time interval), and
        // kNewPosition is the reference result for the new position (end of the time interval)
        final var k1 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldX, oldY, oldZ);

        final var k2 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldX, oldY, oldZ);

        final var k3 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds,
                newEciFrame, oldC, oldVx, oldVy, oldVz);

        final var k4 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                newEciFrame, oldC, oldVx, oldVy, oldVz);

        final var k5 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds,
                newEciFrame, oldEciFrame);

        final var k6 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                newEciFrame, oldEciFrame);

        final var k7 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldX, oldY, oldZ);

        final var k8 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldX, oldY, oldZ);

        final var k9 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds,
                newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final var k10 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final var k11 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldPosition);

        final var k12 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, oldPosition);

        final var k13 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldPosition);

        final var k14 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, oldPosition);

        final var kNewPosition = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(seconds,
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        // both reference results are different because the body has moved
        assertNotEquals(k1, kNewPosition);

        // overloads receiving an explicit (old) position
        assertEquals(k1, k2);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);

        // overloads receiving the old frame use the old position
        assertEquals(k1, k5);
        assertEquals(k1, k6);

        // overloads receiving only the new frame use the new position
        assertEquals(kNewPosition, k3);
        assertEquals(kNewPosition, k4);
        assertEquals(kNewPosition, k9);
        assertEquals(kNewPosition, k10);

        // compare against the in-test reference implementation
        final var expected = estimateKinematics(seconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                oldX, oldY, oldZ);
        assertTrue(k1.equals(expected, ABSOLUTE_ERROR));

        final var expectedNewPosition = estimateKinematics(seconds, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);
        assertTrue(kNewPosition.equals(expectedNewPosition, ABSOLUTE_ERROR));
    }

    private static NEDFrame createOldNedFrame() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {
        return createOldNedFrame(new UniformRandomizer());
    }

    /**
     * Creates a NED frame at the start of the time interval having random velocity and attitude.
     *
     * @param randomizer randomizer used to generate velocity and attitude.
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

    private static NEDFrame createNewNedFrame(final NEDFrame oldFrame) throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {
        return createNewNedFrame(new UniformRandomizer(), oldFrame, MAX_ANGLE_VARIATION_DEGREES);
    }

    /**
     * Creates a NED frame at the end of the time interval by randomly varying position, velocity and
     * attitude of the frame at the start of the time interval.
     *
     * @param randomizer               randomizer used to generate variations.
     * @param oldFrame                 NED frame at the start of the time interval.
     * @param maxAngleVariationDegrees maximum absolute variation, expressed in degrees, of each Euler angle.
     * @return NED frame at the end of the time interval.
     * @throws InvalidRotationMatrixException                never happens.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     */
    private static NEDFrame createNewNedFrame(
            final UniformRandomizer randomizer, final NEDFrame oldFrame, final double maxAngleVariationDegrees)
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

    /**
     * Creates the fixed ECI frame at the start of the time interval used by the round trip tests.
     *
     * @param timeInterval time elapsed since ECI and ECEF frames were coincident, expressed in seconds (s).
     * @return ECI frame at the start of the time interval.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private static ECIFrame createRoundTripOldEciFrame(final double timeInterval)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var q = new Quaternion(Math.toRadians(ROUND_TRIP_ROLL_DEGREES),
                Math.toRadians(ROUND_TRIP_PITCH_DEGREES), Math.toRadians(ROUND_TRIP_YAW_DEGREES));
        final var c = new CoordinateTransformation(q.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT, ROUND_TRIP_VN, ROUND_TRIP_VE, ROUND_TRIP_VD, c);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        return ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(timeInterval, ecefFrame);
    }

    /**
     * Creates the fixed body kinematics used by the round trip tests.
     *
     * @return body kinematics containing a specific force and a non-negligible angular rate.
     */
    private static BodyKinematics createRoundTripKinematics() {
        return new BodyKinematics(ROUND_TRIP_FX, ROUND_TRIP_FY, ROUND_TRIP_FZ,
                ROUND_TRIP_ANGULAR_RATE_X, ROUND_TRIP_ANGULAR_RATE_Y, ROUND_TRIP_ANGULAR_RATE_Z);
    }

    /**
     * Computes the norm of the specific force difference between two body kinematics.
     *
     * @param k1 first body kinematics.
     * @param k2 second body kinematics.
     * @return norm of specific force difference expressed in meters per squared second (m/s^2).
     */
    private static double specificForceDifference(final BodyKinematics k1, final BodyKinematics k2) {
        final var diffX = k1.getFx() - k2.getFx();
        final var diffY = k1.getFy() - k2.getFy();
        final var diffZ = k1.getFz() - k2.getFz();
        return Math.sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
    }

    /**
     * Describes body kinematics so that assertion failures report the observed values.
     *
     * @param k body kinematics to be described.
     * @return text containing specific force and angular rate coordinates.
     */
    private static String describe(final BodyKinematics k) {
        return "f = (" + k.getFx() + ", " + k.getFy() + ", " + k.getFz() + ") m/s^2, w = ("
                + k.getAngularRateX() + ", " + k.getAngularRateY() + ", " + k.getAngularRateZ() + ") rad/s";
    }

    private static BodyKinematics estimateKinematics(
            final double timeInterval, final CoordinateTransformation c, final CoordinateTransformation oldC,
            final double vx, final double vy, final double vz,
            final double oldVx, final double oldVy, final double oldVz, final double x, final double y, final double z)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {

        if (timeInterval > 0.0) {
            // Get coordinate transformation matrix from the old attitude to the new
            final var cBi = c.getMatrix();
            final var oldCbi = oldC.getMatrix();
            final var cOldNew = cBi.transposeAndReturnNew().multiplyAndReturnNew(oldCbi);

            // Calculate the approximate angular rate
            final var alphaIbb = new Matrix(3, 1);
            alphaIbb.setElementAtIndex(0, 0.5 * (cOldNew.getElementAt(1, 2)
                    - cOldNew.getElementAt(2, 1)));
            alphaIbb.setElementAtIndex(1, 0.5 * (cOldNew.getElementAt(2, 0)
                    - cOldNew.getElementAt(0, 2)));
            alphaIbb.setElementAtIndex(2, 0.5 * (cOldNew.getElementAt(0, 1)
                    - cOldNew.getElementAt(1, 0)));

            // Calculate and apply the scaling factor
            final var temp = Math.acos(0.5 * (Utils.trace(cOldNew) - 1.0));
            if (temp > SCALING_THRESHOLD) {
                // scaling is 1 if temp is less than this
                alphaIbb.multiplyByScalar(temp / Math.sin(temp));
            }

            // Calculate the angular rate
            final var omegaIbb = alphaIbb.multiplyByScalarAndReturnNew(1.0 / timeInterval);

            // Calculate the specific force resolved about ECI-frame axes
            // From (5.18) and (5.20)
            final var vIbi = new Matrix(3, 1);
            vIbi.setElementAtIndex(0, vx);
            vIbi.setElementAtIndex(1, vy);
            vIbi.setElementAtIndex(2, vz);

            final var oldVibi = new Matrix(3, 1);
            oldVibi.setElementAtIndex(0, oldVx);
            oldVibi.setElementAtIndex(1, oldVy);
            oldVibi.setElementAtIndex(2, oldVz);

            final var gravitation = ECIGravitationEstimator.estimateGravitationAndReturnNew(x, y, z);
            final var g = gravitation.asMatrix();

            final var fIbi = vIbi.subtractAndReturnNew(oldVibi).multiplyByScalarAndReturnNew(1.0 / timeInterval)
                    .subtractAndReturnNew(g);

            // Calculate the average body to ECI frame coordinate transformation
            // matrix over the update interval using (5.84)
            final var magAlpha = Utils.normF(alphaIbb);
            final var skewAlpha = Utils.skewMatrix(alphaIbb);
            final Matrix aveCbi;
            if (magAlpha > ALPHA_THRESHOLD) {
                final var magAlpha2 = Math.pow(magAlpha, 2.0);
                aveCbi = oldCbi.multiplyAndReturnNew(Matrix.identity(3, 3)
                        .addAndReturnNew(skewAlpha.multiplyByScalarAndReturnNew(
                                (1.0 - Math.cos(magAlpha)) / magAlpha2)).addAndReturnNew(
                                skewAlpha.multiplyAndReturnNew(skewAlpha)
                                        .multiplyByScalarAndReturnNew(
                                                (1.0 - Math.sin(magAlpha) / magAlpha) / magAlpha2)));
            } else {
                aveCbi = oldCbi;
            }

            // Transform specific force to body-frame resolving axes using (5.81)
            final var fIbb = Utils.inverse(aveCbi).multiplyAndReturnNew(fIbi);

            final var fx = fIbb.getElementAtIndex(0);
            final var fy = fIbb.getElementAtIndex(1);
            final var fz = fIbb.getElementAtIndex(2);

            final var angularRateX = omegaIbb.getElementAtIndex(0);
            final var angularRateY = omegaIbb.getElementAtIndex(1);
            final var angularRateZ = omegaIbb.getElementAtIndex(2);

            return new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        } else {
            // If a time interval is zero, set an angular rate and specific force to zer
            return new BodyKinematics();
        }
    }
}
