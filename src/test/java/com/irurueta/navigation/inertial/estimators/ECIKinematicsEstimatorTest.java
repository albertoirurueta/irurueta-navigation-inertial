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
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ECIKinematicsEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double MIN_ANGLE_VARIATION_DEGREES = -5.0;
    private static final double MAX_ANGLE_VARIATION_DEGREES = 5.0;

    private static final double MIN_POSITION_VARIATION_DEGREES = -1e-4;
    private static final double MAX_POSITION_VARIATION_DEGREES = 1e-4;

    private static final double MIN_HEIGHT_VARIATION = -0.5;
    private static final double MAX_HEIGHT_VARIATION = 0.5;

    private static final double MIN_VELOCITY_VARIATION = -0.1;
    private static final double MAX_VELOCITY_VARIATION = 0.1;

    private static final double SCALING_THRESHOLD = 2e-5;
    private static final double ALPHA_THRESHOLD = 1e-8;

    private static final double ABSOLUTE_ERROR = 1e-5;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    @Test
    void testEstimate() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

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

        final var estimator = new ECIKinematicsEstimator();

        final var k1 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var k2 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final var k3 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEciFrame, oldC, oldVx, oldVy, oldVz, k3);

        final var k4 = new BodyKinematics();
        estimator.estimate(timeInterval, newEciFrame, oldC, oldVx, oldVy, oldVz, k4);

        final var k5 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEciFrame, oldEciFrame, k5);

        final var k6 = new BodyKinematics();
        estimator.estimate(timeInterval, newEciFrame, oldEciFrame, k6);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var k7 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ,
                x, y, z, k7);

        final var k8 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k8);

        final var k9 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k9);

        final var k10 = new BodyKinematics();
        estimator.estimate(timeInterval, newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k10);

        final var position = new InhomogeneousPoint3D(x, y, z);
        final var k11 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, k11);

        final var k12 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, k12);

        final var k13 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ,
                position, k13);

        final var k14 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position,
                k14);

        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k9);
        assertEquals(k1, k10);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
    }

    @Test
    void testEstimateAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

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

        final var estimator = new ECIKinematicsEstimator();

        final var k1 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var k2 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final var k3 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame, oldC, oldVx, oldVy, oldVz);

        final var k4 = estimator.estimateAndReturnNew(timeInterval, newEciFrame, oldC, oldVx, oldVy, oldVz);

        final var k5 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame, oldEciFrame);

        final var k6 = estimator.estimateAndReturnNew(timeInterval, newEciFrame, oldEciFrame);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var k7 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final var k8 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final var k9 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final var k10 = estimator.estimateAndReturnNew(timeInterval, newEciFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final var position = new InhomogeneousPoint3D(x, y, z);
        final var k11 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position);

        final var k12 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position);

        final var k13 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final var k14 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position);

        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k9);
        assertEquals(k1, k10);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
    }

    @Test
    void testEstimateKinematics() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException,
            WrongSizeException, RankDeficientMatrixException, DecomposerException {

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

        final var k1 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, k1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var k2 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final var k3 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEciFrame, oldC, oldVx, oldVy, oldVz, k3);

        final var k4 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval, newEciFrame, oldC, oldVx, oldVy, oldVz, k4);

        final var k5 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEciFrame, oldEciFrame, k5);

        final var k6 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval, newEciFrame, oldEciFrame, k6);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var k7 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k7);

        final var k8 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k8);

        final var k9 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEciFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, k9);

        final var k10 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval, newEciFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                k10);

        final var position = new InhomogeneousPoint3D(x, y, z);
        final var k11 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position, k11);

        final var k12 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position,
                k12);

        final var k13 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ, oldSpeedX,
                oldSpeedY, oldSpeedZ, position, k13);

        final var k14 = new BodyKinematics();
        ECIKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position, k14);

        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k9);
        assertEquals(k1, k10);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);

        final var k = estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        assertTrue(k1.equals(k, ABSOLUTE_ERROR));
    }

    @Test
    void testEstimateKinematicsAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException, DecomposerException {

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

        final var k1 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var k2 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final var k3 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame, oldC,
                oldVx, oldVy, oldVz);

        final var k4 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEciFrame, oldC,
                oldVx, oldVy, oldVz);

        final var k5 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame,
                oldEciFrame);

        final var k6 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEciFrame,
                oldEciFrame);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final var k7 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final var k8 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final var k9 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final var k10 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEciFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final var position = new InhomogeneousPoint3D(x, y, z);
        final var k11 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position);

        final var k12 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, position);

        final var k13 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final var k14 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k9);
        assertEquals(k1, k10);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);

        final var k = estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        assertTrue(k1.equals(k, ABSOLUTE_ERROR));
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

    private static NEDFrame createOldNedFrame() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();

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

        final var randomizer = new UniformRandomizer();

        final var latitudeVariation = Math.toRadians(randomizer.nextDouble(MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final var longitudeVariation = Math.toRadians(randomizer.nextDouble(MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final var heightVariation = randomizer.nextDouble(MIN_HEIGHT_VARIATION, MAX_HEIGHT_VARIATION);

        final var vnVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);
        final var veVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);
        final var vdVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);

        final var rollVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, 
                MAX_ANGLE_VARIATION_DEGREES));
        final var pitchVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                MAX_ANGLE_VARIATION_DEGREES));
        final var yawVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                MAX_ANGLE_VARIATION_DEGREES));

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
