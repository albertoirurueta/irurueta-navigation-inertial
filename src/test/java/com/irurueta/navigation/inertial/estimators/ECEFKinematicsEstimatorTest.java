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
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static com.irurueta.navigation.frames.CoordinateTransformation.ROWS;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ECEFKinematicsEstimatorTest {

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

    private static final double ABSOLUTE_ERROR = 1e-4;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    @Test
    public void testEstimate() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final CoordinateTransformation c = newEcefFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEcefFrame.getCoordinateTransformation();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final ECEFKinematicsEstimator estimator = new ECEFKinematicsEstimator();

        final BodyKinematics k1 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final BodyKinematics k2 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final ECEFVelocity velocity = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity oldVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final BodyKinematics k3 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, x, y, z, k3);

        final BodyKinematics k4 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, x, y, z, k4);

        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        final BodyKinematics k5 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition, k5);

        final BodyKinematics k6 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition, k6);

        final BodyKinematics k7 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, ecefPosition, k7);

        final BodyKinematics k8 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, ecefPosition, k8);

        final BodyKinematics k9 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEcefFrame, oldC, oldVx, oldVy, oldVz, k9);

        final BodyKinematics k10 = new BodyKinematics();
        estimator.estimate(timeInterval, newEcefFrame, oldC, oldVx, oldVy, oldVz, k10);

        final BodyKinematics k11 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEcefFrame, oldC, oldVelocity, k11);

        final BodyKinematics k12 = new BodyKinematics();
        estimator.estimate(timeInterval, newEcefFrame, oldC, oldVelocity, k12);

        final BodyKinematics k13 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame, k13);

        final BodyKinematics k14 = new BodyKinematics();
        estimator.estimate(timeInterval, newEcefFrame, oldEcefFrame, k14);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final BodyKinematics k15 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ,
                x, y, z, k15);

        final BodyKinematics k16 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z,
                k16);

        final BodyKinematics k17 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ,
                ecefPosition, k17);

        final BodyKinematics k18 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition,
                k18);

        final BodyKinematics k19 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEcefFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k19);

        final BodyKinematics k20 = new BodyKinematics();
        estimator.estimate(timeInterval, newEcefFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k20);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final BodyKinematics k21 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, k21);

        final BodyKinematics k22 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, k22);

        final BodyKinematics k23 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, position, k23);

        final BodyKinematics k24 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, position, k24);

        final BodyKinematics k25 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ,
                position, k25);

        final BodyKinematics k26 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position,
                k26);

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
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k19);
        assertEquals(k1, k20);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);
    }

    @Test
    public void testEstimateAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformation c = newEcefFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEcefFrame.getCoordinateTransformation();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final ECEFKinematicsEstimator estimator = new ECEFKinematicsEstimator();

        final BodyKinematics k1 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final BodyKinematics k2 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);

        final ECEFVelocity velocity = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity oldVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final BodyKinematics k3 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                x, y, z);

        final BodyKinematics k4 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity, x, y, z);

        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        final BodyKinematics k5 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, ecefPosition);

        final BodyKinematics k6 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                ecefPosition);

        final BodyKinematics k7 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                ecefPosition);

        final BodyKinematics k8 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity,
                ecefPosition);

        final BodyKinematics k9 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, oldC,
                oldVx, oldVy, oldVz);

        final BodyKinematics k10 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame, oldC,
                oldVx, oldVy, oldVz);

        final BodyKinematics k11 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, oldC,
                oldVelocity);

        final BodyKinematics k12 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame, oldC, oldVelocity);

        final BodyKinematics k13 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

        final BodyKinematics k14 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame, oldEcefFrame);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final BodyKinematics k15 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final BodyKinematics k16 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final BodyKinematics k17 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition);

        final BodyKinematics k18 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition);

        final BodyKinematics k19 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final BodyKinematics k20 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final BodyKinematics k21 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, position);

        final BodyKinematics k22 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, position);

        final BodyKinematics k23 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                position);

        final BodyKinematics k24 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity,
                position);

        final BodyKinematics k25 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final BodyKinematics k26 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedX, speedY, speedZ,
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
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k19);
        assertEquals(k1, k20);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);
    }

    @Test
    public void testEstimateKinematics() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException, DecomposerException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final CoordinateTransformation c = newEcefFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEcefFrame.getCoordinateTransformation();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final BodyKinematics k1 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, k1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final BodyKinematics k2 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final ECEFVelocity velocity = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity oldVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final BodyKinematics k3 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, x, y, z, k3);

        final BodyKinematics k4 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, x, y, z, k4);

        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        final BodyKinematics k5 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                ecefPosition, k5);

        final BodyKinematics k6 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition,
                k6);

        final BodyKinematics k7 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, ecefPosition,
                k7);

        final BodyKinematics k8 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, ecefPosition, k8);

        final BodyKinematics k9 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldC, oldVx, oldVy, oldVz, k9);

        final BodyKinematics k10 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame, oldC, oldVx, oldVy, oldVz, k10);

        final BodyKinematics k11 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldC, oldVelocity, k11);

        final BodyKinematics k12 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame, oldC, oldVelocity, k12);

        final BodyKinematics k13 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame, k13);

        final BodyKinematics k14 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame, oldEcefFrame, k14);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final BodyKinematics k15 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k15);

        final BodyKinematics k16 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k16);

        final BodyKinematics k17 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition, k17);

        final BodyKinematics k18 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition, k18);

        final BodyKinematics k19 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, k19);

        final BodyKinematics k20 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                k20);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final BodyKinematics k21 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position, k21);

        final BodyKinematics k22 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position,
                k22);

        final BodyKinematics k23 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, position,
                k23);

        final BodyKinematics k24 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, position, k24);

        final BodyKinematics k25 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position, k25);

        final BodyKinematics k26 = new BodyKinematics();
        ECEFKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedX, speedY, speedZ,
                oldSpeedX, oldSpeedY, oldSpeedZ, position, k26);

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
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k19);
        assertEquals(k1, k20);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);

        final BodyKinematics k = estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);

        assertTrue(k1.equals(k, ABSOLUTE_ERROR));
    }

    @Test
    public void testEstimateKinematicsAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException, DecomposerException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final CoordinateTransformation c = newEcefFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEcefFrame.getCoordinateTransformation();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final BodyKinematics k1 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final BodyKinematics k2 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final ECEFVelocity velocity = new ECEFVelocity(vx, vy, vz);
        final ECEFVelocity oldVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final BodyKinematics k3 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                velocity, oldVelocity, x, y, z);

        final BodyKinematics k4 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                velocity, oldVelocity, x, y, z);

        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        final BodyKinematics k5 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition);

        final BodyKinematics k6 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, ecefPosition);

        final BodyKinematics k7 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                velocity, oldVelocity, ecefPosition);

        final BodyKinematics k8 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                velocity, oldVelocity, ecefPosition);

        final BodyKinematics k9 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame, oldC, oldVx, oldVy, oldVz);

        final BodyKinematics k10 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEcefFrame,
                oldC, oldVx, oldVy, oldVz);

        final BodyKinematics k11 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame, oldC, oldVelocity);

        final BodyKinematics k12 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEcefFrame,
                oldC, oldVelocity);

        final BodyKinematics k13 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame, oldEcefFrame);

        final BodyKinematics k14 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEcefFrame,
                oldEcefFrame);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final BodyKinematics k15 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c,
                oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final BodyKinematics k16 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final BodyKinematics k17 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c,
                oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition);

        final BodyKinematics k18 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, ecefPosition);

        final BodyKinematics k19 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame, oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final BodyKinematics k20 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, newEcefFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final BodyKinematics k21 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c,
                oldC, vx, vy, vz, oldVx, oldVy, oldVz, position);

        final BodyKinematics k22 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position);

        final BodyKinematics k23 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c,
                oldC, velocity, oldVelocity, position);

        final BodyKinematics k24 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                velocity, oldVelocity, position);

        final BodyKinematics k25 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c,
                oldC, speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final BodyKinematics k26 = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
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
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k19);
        assertEquals(k1, k20);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);

        final BodyKinematics k = estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);

        assertTrue(k1.equals(k, ABSOLUTE_ERROR));
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenNegativeIntervalThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final CoordinateTransformation c = newEcefFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEcefFrame.getCoordinateTransformation();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(-1.0, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenInvalidCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final CoordinateTransformation c = newEcefFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEcefFrame.getCoordinateTransformation();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        c.setDestinationType(FrameType.BODY_FRAME);
        ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenInvalidOldCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final CoordinateTransformation c = newEcefFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEcefFrame.getCoordinateTransformation();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        oldC.setDestinationType(FrameType.BODY_FRAME);
        ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z);
    }

    @Test
    public void testEstimateKinematicsWhenZeroTimeIntervalReturnsZeroValues()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);

        final CoordinateTransformation c = newEcefFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEcefFrame.getCoordinateTransformation();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final BodyKinematics k = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(0.0, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        assertEquals(0.0, k.getFx(), 0.0);
        assertEquals(0.0, k.getFy(), 0.0);
        assertEquals(0.0, k.getFz(), 0.0);

        assertEquals(0.0, k.getAngularRateX(), 0.0);
        assertEquals(0.0, k.getAngularRateY(), 0.0);
        assertEquals(0.0, k.getAngularRateZ(), 0.0);

        final BodyKinematics k2 = estimateKinematics(0.0, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        assertTrue(k2.equals(k, 0.0));
    }

    @Test
    public void testCompareKinematics() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {
        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                oldEcefFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
        final ECIFrame newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame);

        final BodyKinematics nedK = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                newNedFrame, oldNedFrame);
        final BodyKinematics ecefK = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                newEcefFrame, oldEcefFrame);
        final BodyKinematics eciK = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                newEciFrame, oldEciFrame);

        final double nedSpecificForceNorm = nedK.getSpecificForceNorm();
        final double ecefSpecificForceNorm = ecefK.getSpecificForceNorm();
        final double eciSpecificForceNorm = eciK.getSpecificForceNorm();

        final double nedAngularRateNorm = nedK.getAngularRateNorm();
        final double ecefAngularRateNorm = ecefK.getAngularRateNorm();
        final double eciAngularRateNorm = eciK.getAngularRateNorm();

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

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double longitude = Math.toRadians(LONGITUDE_DEGREES);
        return new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
    }

    private static NEDFrame createNewNedFrame(final NEDFrame oldFrame) throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final double oldLatitude = oldFrame.getLatitude();
        final double oldLongitude = oldFrame.getLongitude();
        final double oldHeight = oldFrame.getHeight();

        final double oldVn = oldFrame.getVn();
        final double oldVe = oldFrame.getVe();
        final double oldVd = oldFrame.getVd();

        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

        final double oldRoll = oldC.getRollEulerAngle();
        final double oldPitch = oldC.getPitchEulerAngle();
        final double oldYaw = oldC.getYawEulerAngle();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitudeVariation = Math.toRadians(randomizer.nextDouble(MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final double longitudeVariation = Math.toRadians(randomizer.nextDouble(MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final double heightVariation = randomizer.nextDouble(MIN_HEIGHT_VARIATION, MAX_HEIGHT_VARIATION);

        final double vnVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);
        final double veVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);
        final double vdVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);

        final double rollVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                MAX_ANGLE_VARIATION_DEGREES));
        final double pitchVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                MAX_ANGLE_VARIATION_DEGREES));
        final double yawVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                MAX_ANGLE_VARIATION_DEGREES));

        final double latitude = oldLatitude + latitudeVariation;
        final double longitude = oldLongitude + longitudeVariation;
        final double height = oldHeight + heightVariation;

        final double vn = oldVn + vnVariation;
        final double ve = oldVe + veVariation;
        final double vd = oldVd + vdVariation;

        final double roll = oldRoll + rollVariation;
        final double pitch = oldPitch + pitchVariation;
        final double yaw = oldYaw + yawVariation;

        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
    }

    private static BodyKinematics estimateKinematics(
            final double timeInterval, final CoordinateTransformation c, final CoordinateTransformation oldC,
            final double vx, final double vy, final double vz,
            final double oldVx, final double oldVy, final double oldVz, final double x, final double y, final double z)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {

        if (timeInterval > 0.0) {
            // From (2.145) determine the Earth rotation over the update interval
            final double omegaIe = Constants.EARTH_ROTATION_RATE;
            final double alphaIe = omegaIe * timeInterval;
            final Matrix cEarth = CoordinateTransformation.eciToEcefMatrixFromAngle(alphaIe);
            final Matrix cBe = c.getMatrix();
            final Matrix oldCbe = oldC.getMatrix();
            final Matrix cOldNew = cBe.transposeAndReturnNew().multiplyAndReturnNew(
                    cEarth.multiplyAndReturnNew(oldCbe));

            // Calculate the approximate angular rate with respect an inertial frame
            final Matrix alphaIbb = new Matrix(ROWS, 1);
            alphaIbb.setElementAtIndex(0,
                    0.5 * (cOldNew.getElementAt(1, 2) - cOldNew.getElementAt(2, 1)));
            alphaIbb.setElementAtIndex(1,
                    0.5 * (cOldNew.getElementAt(2, 0) - cOldNew.getElementAt(0, 2)));
            alphaIbb.setElementAtIndex(2,
                    0.5 * (cOldNew.getElementAt(0, 1) - cOldNew.getElementAt(1, 0)));

            // Calculate and apply the scaling factor
            final double temp = Math.acos(0.5 * (cOldNew.getElementAt(0, 0) +
                    cOldNew.getElementAt(1, 1) + cOldNew.getElementAt(2, 2) - 1.0));
            if (temp > SCALING_THRESHOLD) {
                // scaling is 1 if temp is less than this
                alphaIbb.multiplyByScalar(temp / Math.sin(temp));
            }

            // Calculate the angular rate
            final Matrix omegaIbb = alphaIbb.multiplyByScalarAndReturnNew(1.0 / timeInterval);

            // Calculate the specific force resolved about ECEF-frame axes
            // Frame (5.36)
            final Matrix vEbe = new Matrix(ROWS, 1);
            vEbe.setElementAtIndex(0, vx);
            vEbe.setElementAtIndex(1, vy);
            vEbe.setElementAtIndex(2, vz);

            final Matrix oldVebe = new Matrix(ROWS, 1);
            oldVebe.setElementAtIndex(0, oldVx);
            oldVebe.setElementAtIndex(1, oldVy);
            oldVebe.setElementAtIndex(2, oldVz);

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(x, y, z);
            final Matrix g = gravity.asMatrix();

            final Matrix fIbe = (vEbe.subtractAndReturnNew(oldVebe).multiplyByScalarAndReturnNew(1.0 / timeInterval)
                    .subtractAndReturnNew(g).addAndReturnNew(Utils.skewMatrix(new double[]{0.0, 0.0, omegaIe})
                            .multiplyByScalarAndReturnNew(2.0).multiplyAndReturnNew(oldVebe)));

            // Calculate the average body-to-ECEF-frame coordinate transformation
            // matrix over the update interval using (5,84) and (5.85)
            final double magAlpha = Utils.normF(alphaIbb);
            final Matrix skewAlphaIbb = Utils.skewMatrix(alphaIbb);
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
            final Matrix fIbb = Utils.inverse(aveCbe).multiplyAndReturnNew(fIbe);

            final double specificForceX = fIbb.getElementAtIndex(0);
            final double specificForceY = fIbb.getElementAtIndex(1);
            final double specificForceZ = fIbb.getElementAtIndex(2);

            final double angularRateX = omegaIbb.getElementAtIndex(0);
            final double angularRateY = omegaIbb.getElementAtIndex(1);
            final double angularRateZ = omegaIbb.getElementAtIndex(2);

            // save result data
            return new BodyKinematics(specificForceX, specificForceY, specificForceZ,
                    angularRateX, angularRateY, angularRateZ);
        } else {
            // If time interval is zero, set angular rate and specific force to zero
            return new BodyKinematics();
        }
    }

}
