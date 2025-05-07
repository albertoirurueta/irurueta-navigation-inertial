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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class LevelingEstimatorTest {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_HEIGHT = -100.0;
    private static final double MAX_HEIGHT = 100.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testCheckKinematicValues() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fx1 = kinematics.getFx();
        final var fy1 = kinematics.getFy();
        final var fz1 = kinematics.getFz();

        // check that kinematics values are correct

        // get NED to body conversion matrix
        final var cnb = bodyC.getMatrix();

        final var nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);
        final var g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final var f2 = cnb.multiplyAndReturnNew(g);

        final var fx2 = f2.getElementAtIndex(0);
        final var fy2 = f2.getElementAtIndex(1);
        final var fz2 = f2.getElementAtIndex(2);

        final var f3 = Matrix.newFromArray(new double[]{Math.sin(pitch) * nedGravity.getGd(),
                -Math.cos(pitch) * Math.sin(roll) * nedGravity.getGd(),
                -Math.cos(pitch) * Math.cos(roll) * nedGravity.getGd()
        });

        final var fx3 = f3.getElementAtIndex(0);
        final var fy3 = f3.getElementAtIndex(1);
        final var fz3 = f3.getElementAtIndex(2);

        assertEquals(fx1, fx2, LARGE_ABSOLUTE_ERROR);
        assertEquals(fx2, fx3, ABSOLUTE_ERROR);

        assertEquals(fy1, fy2, LARGE_ABSOLUTE_ERROR);
        assertEquals(fy2, fy3, ABSOLUTE_ERROR);

        assertEquals(fz1, fz2, LARGE_ABSOLUTE_ERROR);
        assertEquals(fz2, fz3, ABSOLUTE_ERROR);
    }

    @Test
    void testGetRollWhenNoRotationTheoretical() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = 0.0;
        final var pitch1 = 0.0;
        final var yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        // get specific force neglecting the north component of gravity
        final var cnb = bodyC.getMatrix();
        final var nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);
        final var g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final var f = cnb.multiplyAndReturnNew(g);

        final var fy = f.getElementAtIndex(1);
        final var fz = f.getElementAtIndex(2);

        final var roll2 = LevelingEstimator.getRoll(fy, fz);

        // Because we have only taken gravity term in measured specific
        // force, the amount of error is smaller. However, in real
        // environments, measured specific force will contain a term due
        // to Earth rotation, and additional terms due to different kinds
        // of errors (bias, scaling, noise, etc).
        assertEquals(roll1, roll2, ABSOLUTE_ERROR);
    }

    @Test
    void testGetRollWhenNoRotation() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = 0.0;
        final var pitch1 = 0.0;
        final var yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME, 
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = LevelingEstimator.getRoll(kinematics.getFy(), kinematics.getFz());

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetRollWithRotationTheoretical() throws WrongSizeException {
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

        // get specific force neglecting the north component of gravity
        final var cnb = bodyC.getMatrix();
        final var nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);
        final var g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final var f = cnb.multiplyAndReturnNew(g);

        final var fy = f.getElementAtIndex(1);
        final var fz = f.getElementAtIndex(2);

        final var roll2 = LevelingEstimator.getRoll(fy, fz);

        // Because we have only taken gravity term in measured specific
        // force, the amount of error is smaller. However, in real
        // environments, measured specific force will contain a term due
        // to Earth rotation, and additional terms due to different kinds
        // of errors (bias, scaling, noise, etc).
        assertEquals(roll1, roll2, ABSOLUTE_ERROR);
    }

    @Test
    void testGetRollWithRotation() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = LevelingEstimator.getRoll(kinematics.getFy(), kinematics.getFz());

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetPitchWhenNoRotationTheoretical() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = 0.0;
        final var pitch1 = 0.0;
        final var yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        // get specific force neglecting the north component of gravity
        final var cnb = bodyC.getMatrix();
        final var nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);
        final var g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final var f = cnb.multiplyAndReturnNew(g);

        final var fx = f.getElementAtIndex(0);
        final var fy = f.getElementAtIndex(1);
        final var fz = f.getElementAtIndex(2);

        final var pitch2 = LevelingEstimator.getPitch(fx, fy, fz);

        // Because we have only taken gravity term in measured specific
        // force, the amount of error is smaller. However, in real
        // environments, measured specific force will contain a term due
        // to Earth rotation, and additional terms due to different kinds
        // of errors (bias, scaling, noise, etc).
        assertEquals(pitch1, pitch2, ABSOLUTE_ERROR);
    }

    @Test
    void testGetPitchWhenNoRotation() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = 0.0;
        final var pitch1 = 0.0;
        final var yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var pitch2 = LevelingEstimator.getPitch(kinematics.getFx(), kinematics.getFy(), kinematics.getFz());

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetPitchWithRotationTheoretical() throws WrongSizeException {
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

        // get specific force neglecting the north component of gravity
        final var cnb = bodyC.getMatrix();
        final var nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);
        final var g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final var f = cnb.multiplyAndReturnNew(g);

        final var fx = f.getElementAtIndex(0);
        final var fy = f.getElementAtIndex(1);
        final var fz = f.getElementAtIndex(2);

        final var pitch2 = LevelingEstimator.getPitch(fx, fy, fz);

        // Because we have only taken gravity term in measured specific
        // force, the amount of error is smaller. However, in real
        // environments, measured specific force will contain a term due
        // to Earth rotation, and additional terms due to different kinds
        // of errors (bias, scaling, noise, etc).
        assertEquals(pitch1, pitch2, ABSOLUTE_ERROR);
    }

    @Test
    void testGetPitchWithRotation() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var pitch2 = LevelingEstimator.getPitch(kinematics.getFx(), kinematics.getFy(), kinematics.getFz());

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetYawWhenNoRotation() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final var roll1 = 0.0;
        final var pitch1 = 0.0;
        final var yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = LevelingEstimator.getRoll(kinematics.getFy(), kinematics.getFz());
        final var pitch2 = LevelingEstimator.getPitch(kinematics.getFx(), kinematics.getFy(), kinematics.getFz());
        final var yaw2 = LevelingEstimator.getYaw(roll2, pitch2, 
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetYawWithRotation() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = LevelingEstimator.getRoll(kinematics.getFy(), kinematics.getFz());
        final var pitch2 = LevelingEstimator.getPitch(kinematics.getFx(), kinematics.getFy(), kinematics.getFz());
        final var yaw2 = LevelingEstimator.getYaw(roll2, pitch2, 
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetYaw2() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var yaw2 = LevelingEstimator.getYaw(kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetRoll2() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = LevelingEstimator.getRoll(kinematics);

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetPitch2() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var pitch2 = LevelingEstimator.getPitch(kinematics);

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetYaw3() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var yaw2 = LevelingEstimator.getYaw(kinematics);

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        LevelingEstimator.getAttitude(kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ(), bodyC2);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude2() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        LevelingEstimator.getAttitude(kinematics, bodyC2);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = LevelingEstimator.getAttitude(
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude4() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var bodyC2 = LevelingEstimator.getAttitude(kinematics);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetRoll3() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();

        final var roll2 = LevelingEstimator.getRoll(fy, fz);

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetPitch3() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();

        final var pitch2 = LevelingEstimator.getPitch(fx, fy, fz);

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetYaw4() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = new Angle(LevelingEstimator.getRoll(kinematics), AngleUnit.RADIANS);
        final var pitch2 = new Angle(LevelingEstimator.getPitch(kinematics), AngleUnit.RADIANS);
        final var angularRateX = kinematics.getAngularSpeedX();
        final var angularRateY = kinematics.getAngularSpeedY();
        final var angularRateZ = kinematics.getAngularSpeedZ();

        final var yaw2 = LevelingEstimator.getYaw(roll2, pitch2, angularRateX, angularRateY, angularRateZ);

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testGetYaw5() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();
        final var angularRateX = kinematics.getAngularSpeedX();
        final var angularRateY = kinematics.getAngularSpeedY();
        final var angularRateZ = kinematics.getAngularSpeedZ();

        final var yaw2 = LevelingEstimator.getYaw(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        // Kinematics estimator also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term. For that reason the amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();
        final var angularRateX = kinematics.getAngularSpeedX();
        final var angularRateY = kinematics.getAngularSpeedY();
        final var angularRateZ = kinematics.getAngularSpeedZ();

        final var bodyC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        LevelingEstimator.getAttitude(fx, fy, fz, angularRateX, angularRateY, angularRateZ, bodyC2);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetAttitude6() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();
        final var angularRateX = kinematics.getAngularSpeedX();
        final var angularRateY = kinematics.getAngularSpeedY();
        final var angularRateZ = kinematics.getAngularSpeedZ();

        final var bodyC2 = LevelingEstimator.getAttitude(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetRollAsAngle() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = LevelingEstimator.getRoll(kinematics.getFy(), kinematics.getFz());
        final var roll3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getRollAsAngle(kinematics.getFy(), kinematics.getFz(), roll3);
        final var roll4 = LevelingEstimator.getRollAsAngle(kinematics.getFy(), kinematics.getFz());

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(roll2, roll3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, roll3.getUnit());
        assertEquals(roll3, roll4);
    }

    @Test
    void testGetPitchAsAngle() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var pitch2 = LevelingEstimator.getPitch(kinematics.getFx(), kinematics.getFy(), kinematics.getFz());
        final var pitch3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getPitchAsAngle(kinematics.getFx(), kinematics.getFy(), kinematics.getFz(), pitch3);
        final var pitch4 = LevelingEstimator.getPitchAsAngle(
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz());

        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch2, pitch3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, pitch3.getUnit());
        assertEquals(pitch3, pitch4);
    }

    @Test
    void testGetYawAsAngle() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = LevelingEstimator.getRoll(kinematics.getFy(), kinematics.getFz());
        final var pitch2 = LevelingEstimator.getPitch(kinematics.getFx(), kinematics.getFy(), kinematics.getFz());
        final var yaw2 = LevelingEstimator.getYaw(roll2, pitch2,
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        final var yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(roll2, pitch2, kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), yaw3);
        final var yaw4 = LevelingEstimator.getYawAsAngle(roll2, pitch2, kinematics.getAngularRateX(),
                kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, yaw3.getUnit());
        assertEquals(yaw3, yaw4);
    }

    @Test
    void testGetYawAsAngle2() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var yaw2 = LevelingEstimator.getYaw(kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());
        final var yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ(), yaw3);
        final var yaw4 = LevelingEstimator.getYawAsAngle(kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ());

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, yaw3.getUnit());
        assertEquals(yaw3, yaw4);
    }

    @Test
    void testGetRollAsAngle2() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = LevelingEstimator.getRoll(kinematics);
        final var roll3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getRollAsAngle(kinematics, roll3);
        final var roll4 = LevelingEstimator.getRollAsAngle(kinematics);

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(roll2, roll3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, roll3.getUnit());
        assertEquals(roll3, roll4);
    }

    @Test
    void testGetPitchAsAngle2() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var pitch2 = LevelingEstimator.getPitch(kinematics);
        final var pitch3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getPitchAsAngle(kinematics, pitch3);
        final var pitch4 = LevelingEstimator.getPitchAsAngle(kinematics);

        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch2, pitch3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, pitch3.getUnit());
        assertEquals(pitch3, pitch4);
    }

    @Test
    void testGetYawAsAngle3() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var yaw2 = LevelingEstimator.getYaw(kinematics);
        final var yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(kinematics, yaw3);
        final var yaw4 = LevelingEstimator.getYawAsAngle(kinematics);

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, yaw3.getUnit());
        assertEquals(yaw3, yaw4);
    }

    @Test
    void testGetRollAsAngle3() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();

        final var roll2 = LevelingEstimator.getRoll(fy, fz);
        final var roll3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getRollAsAngle(fy, fz, roll3);
        final var roll4 = LevelingEstimator.getRollAsAngle(fy, fz);

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(roll2, roll3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, roll3.getUnit());
        assertEquals(roll3, roll4);
    }

    @Test
    void testGetPitchAsAngle3() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();

        final var pitch2 = LevelingEstimator.getPitch(fx, fy, fz);
        final var pitch3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getPitchAsAngle(fx, fy, fz, pitch3);
        final var pitch4 = LevelingEstimator.getPitchAsAngle(fx, fy, fz);

        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch2, pitch3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, pitch3.getUnit());
        assertEquals(pitch3, pitch4);
    }

    @Test
    void testGetYawAsAngle4() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var roll2 = new Angle(LevelingEstimator.getRoll(kinematics), AngleUnit.RADIANS);
        final var pitch2 = new Angle(LevelingEstimator.getPitch(kinematics), AngleUnit.RADIANS);
        final var angularRateX = kinematics.getAngularSpeedX();
        final var angularRateY = kinematics.getAngularSpeedY();
        final var angularRateZ = kinematics.getAngularSpeedZ();

        final var yaw2 = LevelingEstimator.getYaw(roll2, pitch2, angularRateX, angularRateY, angularRateZ);
        final var yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(roll2, pitch2, angularRateX, angularRateY, angularRateZ, yaw3);
        final var yaw4 = LevelingEstimator.getYawAsAngle(roll2, pitch2, angularRateX, angularRateY, angularRateZ);

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, yaw3.getUnit());
        assertEquals(yaw3, yaw4);
    }

    @Test
    void testGetYawAsAngle5() {
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

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();
        final var angularRateX = kinematics.getAngularSpeedX();
        final var angularRateY = kinematics.getAngularSpeedY();
        final var angularRateZ = kinematics.getAngularSpeedZ();

        final var yaw2 = LevelingEstimator.getYaw(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        final var yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(fx, fy, fz, angularRateX, angularRateY, angularRateZ, yaw3);
        final var yaw4 = LevelingEstimator.getYawAsAngle(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, yaw3.getUnit());
        assertEquals(yaw3, yaw4);
    }
}
