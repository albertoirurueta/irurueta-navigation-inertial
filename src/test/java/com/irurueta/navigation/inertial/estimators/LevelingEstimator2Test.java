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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

class LevelingEstimator2Test {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT = -100.0;
    private static final double MAX_HEIGHT = 100.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final double ABSOLUTE_ERROR = 1e-4;

    @Test
    void testGetAttitude1() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        LevelingEstimator2.getAttitude(latitude, height, kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ(), bodyC2);

        assertTrue(bodyC.equals(bodyC2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude2() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        // body attitude
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        LevelingEstimator2.getAttitude(nedPosition, kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ(), bodyC2);

        assertTrue(bodyC.equals(bodyC2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude3() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        LevelingEstimator2.getAttitude(latitude, height, kinematics, bodyC2);

        assertTrue(bodyC.equals(bodyC2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude4() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        // body attitude
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        LevelingEstimator2.getAttitude(nedPosition, kinematics, bodyC2);

        assertTrue(bodyC.equals(bodyC2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude5() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = LevelingEstimator2.getAttitude(latitude, height,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        assertTrue(bodyC.equals(bodyC2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude6() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        // body attitude
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = LevelingEstimator2.getAttitude(nedPosition,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        assertTrue(bodyC.equals(bodyC2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude7() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = LevelingEstimator2.getAttitude(latitude, height, kinematics);

        assertTrue(bodyC.equals(bodyC2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude8() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        // body attitude
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = LevelingEstimator2.getAttitude(nedPosition, kinematics);

        assertTrue(bodyC.equals(bodyC2, ABSOLUTE_ERROR));
    }
}
