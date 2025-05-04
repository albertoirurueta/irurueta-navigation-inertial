/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertTrue;

class QuaternionIntegratorTest {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT = -100.0;
    private static final double MAX_HEIGHT = 10000.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    // assume 50 samples per second
    private static final int NUM_SAMPLES = 50;
    private static final double TIME_INTERVAL = 1.0;
    private static final double TIME_INTERVAL_BETWEEN_SAMPLES = TIME_INTERVAL / NUM_SAMPLES;

    private static final double ABSOLUTE_ERROR = 1e-2;

    private static final double LARGE_ABSOLUTE_ERROR = 6e-2;

    private static final int TIMES = 100;

    @Test
    void testIntegrateGyroSequenceWithInitialAttitudeAndEulerType()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, RotationException,
            InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics, 
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var expected2 = ecefFrame2.getCoordinateTransformation().asRotation().toQuaternion();
            expected2.normalize();

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR));

            final var sequence = new BodyKinematicsSequence<>(items);
            final var initialAttitude = ecefFrame1.getCoordinateTransformation().asRotation().toQuaternion();
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, initialAttitude,
                    QuaternionStepIntegratorType.EULER_METHOD, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceWithNoInitialAttitudeAndEulerType()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, RotationException,
            InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity ad initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, null,
                    QuaternionStepIntegratorType.EULER_METHOD, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceWithEulerType() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity ad initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, QuaternionStepIntegratorType.EULER_METHOD, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceAndReturnNewWithEulerType() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var expected2 = ecefFrame2.getCoordinateTransformation().asRotation().toQuaternion();
            expected2.normalize();

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR));

            final var sequence = new BodyKinematicsSequence<>(items);
            final var initialAttitude = ecefFrame1.getCoordinateTransformation().asRotation().toQuaternion();
            final var result = QuaternionIntegrator.integrateGyroSequenceAndReturnNew(sequence, initialAttitude,
                    QuaternionStepIntegratorType.EULER_METHOD);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceAndReturnNewWithEulerType2() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException, InertialNavigatorException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity as initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = QuaternionIntegrator.integrateGyroSequenceAndReturnNew(sequence,
                    QuaternionStepIntegratorType.EULER_METHOD);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceWithInitialAttitudeAndMidPointType()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, RotationException,
            InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var expected2 = ecefFrame2.getCoordinateTransformation().asRotation().toQuaternion();
            expected2.normalize();

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR));

            final var sequence = new BodyKinematicsSequence<>(items);
            final var initialAttitude = ecefFrame1.getCoordinateTransformation().asRotation().toQuaternion();
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, initialAttitude,
                    QuaternionStepIntegratorType.MID_POINT, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceWithNoInitialAttitudeAndMidPointType()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, RotationException,
            InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity ad initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, null,
                    QuaternionStepIntegratorType.MID_POINT, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceWithMidPointType() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity ad initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, QuaternionStepIntegratorType.MID_POINT, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceAndReturnNewWithMidPointType() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var expected2 = ecefFrame2.getCoordinateTransformation().asRotation().toQuaternion();
            expected2.normalize();

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR));

            final var sequence = new BodyKinematicsSequence<>(items);
            final var initialAttitude = ecefFrame1.getCoordinateTransformation().asRotation().toQuaternion();
            final var result = QuaternionIntegrator.integrateGyroSequenceAndReturnNew(sequence, initialAttitude,
                    QuaternionStepIntegratorType.MID_POINT);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceAndReturnNewWithMidPointType2() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException, InertialNavigatorException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity as initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (int i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = QuaternionIntegrator.integrateGyroSequenceAndReturnNew(sequence,
                    QuaternionStepIntegratorType.MID_POINT);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceWithInitialAttitudeAndRungeKuttaType()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, RotationException,
            InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var expected2 = ecefFrame2.getCoordinateTransformation().asRotation().toQuaternion();
            expected2.normalize();

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR));

            final var sequence = new BodyKinematicsSequence<>(items);
            final var initialAttitude = ecefFrame1.getCoordinateTransformation().asRotation().toQuaternion();
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, initialAttitude,
                    QuaternionStepIntegratorType.RUNGE_KUTTA, result);
            result.normalize();

            if (!result.equals(expected, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceWithNoInitialAttitudeAndRungeKuttaType()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, RotationException,
            InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity ad initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, null,
                    QuaternionStepIntegratorType.RUNGE_KUTTA, result);
            result.normalize();

            if (!result.equals(expected, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceWithRungeKuttaType() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity ad initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, QuaternionStepIntegratorType.RUNGE_KUTTA, result);
            result.normalize();

            if (!result.equals(expected, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceAndReturnNewWithRungeKuttaType() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var expected2 = ecefFrame2.getCoordinateTransformation().asRotation().toQuaternion();
            expected2.normalize();

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR));

            final var sequence = new BodyKinematicsSequence<>(items);
            final var initialAttitude = ecefFrame1.getCoordinateTransformation().asRotation().toQuaternion();
            final var result = QuaternionIntegrator.integrateGyroSequenceAndReturnNew(sequence, initialAttitude,
                    QuaternionStepIntegratorType.RUNGE_KUTTA);
            result.normalize();

            if (!result.equals(expected, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testIntegrateGyroSequenceAndReturnNewWithRungeKuttaType2()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, RotationException,
            InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var cbn2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final var kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2,
                    ecefFrame1);

            final var previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity as initial attitude
            previousFrame.setCoordinateTransformation(new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final var currentFrame = new ECEFFrame();

            final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (var i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final var timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final var item = new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final var expected = currentFrame.getCoordinateTransformation().asRotation().toQuaternion();
            expected.normalize();

            final var sequence = new BodyKinematicsSequence<>(items);
            final var result = QuaternionIntegrator.integrateGyroSequenceAndReturnNew(sequence,
                    QuaternionStepIntegratorType.RUNGE_KUTTA);
            result.normalize();

            if (!result.equals(expected, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }
}