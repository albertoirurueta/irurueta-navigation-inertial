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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class INSLooselyCoupledKalmanEpochEstimatorTest {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -50.0;
    private static final double MAX_HEIGHT_METERS = 50.0;

    private static final double MIN_SPEED_VALUE = -2.0;
    private static final double MAX_SPEED_VALUE = 2.0;

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testEstimate() throws AlgebraException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, 
                    MAX_LONGITUDE_DEGREES));
            final var userHeight = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

            final var userVn = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
            final var userVe = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
            final var userVd = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

            final var userNedPosition = new NEDPosition(userLatitude, userLongitude, userHeight);
            final var userNedVelocity = new NEDVelocity(userVn, userVe, userVd);

            final var userEcefPosition = new ECEFPosition();
            final var userEcefVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(userNedPosition, userNedVelocity, userEcefPosition,
                    userEcefVelocity);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var previousPosition = new ECEFPosition(
                    userEcefPosition.getX() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVx(),
                    userEcefPosition.getY() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVy(),
                    userEcefPosition.getZ() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVz());

            final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS, 
                    INSLooselyCoupledKalmanState.NUM_PARAMS);

            final var previousState = new INSLooselyCoupledKalmanState(c, userEcefVelocity, previousPosition, 
                    accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, 
                    covariance);

            final var bodyKinematics = new BodyKinematics();

            final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var config = new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, 
                    accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);

            final var newState1 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, config);

            final var newState2 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, config, newState2);

            final var previousNedPosition = new NEDPosition();
            final var previousNedVelocity = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(previousPosition, userEcefVelocity, previousNedPosition,
                    previousNedVelocity);
            final var previousLatitude = previousNedPosition.getLatitude();

            final var newState3 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitude, config);

            final var newState4 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, previousLatitude, config, newState4);

            final var fx = bodyKinematics.getFx();
            final var fy = bodyKinematics.getFy();
            final var fz = bodyKinematics.getFz();

            final var newState5 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, config);

            final var newState6 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, config, newState6);

            final var newState7 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitude, config);

            final var newState8 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, previousLatitude, config, newState8);

            final var x = userEcefPosition.getX();
            final var y = userEcefPosition.getY();
            final var z = userEcefPosition.getZ();
            final var vx = userEcefVelocity.getVx();
            final var vy = userEcefVelocity.getVy();
            final var vz = userEcefVelocity.getVz();
            final var newState9 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, config);

            final var newState10 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, config, newState10);

            final var newState11 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitude, config);

            final var newState12 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, previousLatitude, config, newState12);

            final var newState13 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, config);

            final var newState14 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, TIME_INTERVAL_SECONDS, previousState,
                    fx, fy, fz, config, newState14);

            final var newState15 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitude, config);

            final var newState16 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, TIME_INTERVAL_SECONDS, previousState,
                    fx, fy, fz, previousLatitude, config, newState16);

            final var propagationInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final var newState17 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    propagationInterval, previousState, bodyKinematics, config);

            final var newState18 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, propagationInterval,
                    previousState, bodyKinematics, config, newState18);

            final var newState19 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    propagationInterval, previousState, bodyKinematics, previousLatitude, config);

            final var newState20 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, propagationInterval,
                    previousState, bodyKinematics, previousLatitude, config, newState20);

            final var newState21 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    propagationInterval, previousState, fx, fy, fz, config);

            final var newState22 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, propagationInterval,
                    previousState, fx, fy, fz, config, newState22);

            final var newState23 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    propagationInterval, previousState, fx, fy, fz, previousLatitude, config);

            final var newState24 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, propagationInterval,
                    previousState, fx, fy, fz, previousLatitude, config, newState24);

            final var newState25 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    propagationInterval, previousState, bodyKinematics, config);

            final var newState26 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                    bodyKinematics, config, newState26);

            final var newState27 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    propagationInterval, previousState, bodyKinematics, previousLatitude, config);

            final var newState28 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                    bodyKinematics, previousLatitude, config, newState28);

            final var newState29 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    propagationInterval, previousState, fx, fy, fz, config);

            final var newState30 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                    fx, fy, fz, config, newState30);

            final var newState31 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    propagationInterval, previousState, fx, fy, fz, previousLatitude, config);

            final var newState32 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                    fx, fy, fz, previousLatitude, config, newState32);

            final var userPosition = new InhomogeneousPoint3D(x, y, z);
            final var newState33 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, config);

            final var newState34 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, config, newState34);

            final var newState35 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitude, config);

            final var newState36 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, previousLatitude, config, newState36);

            final var newState37 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, config);

            final var newState38 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, config, newState38);

            final var newState39 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitude, config);

            final var newState40 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, previousLatitude, config, newState40);

            final var newState41 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    propagationInterval, previousState, bodyKinematics, config);

            final var newState42 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, propagationInterval,
                    previousState, bodyKinematics, config, newState42);

            final var newState43 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    propagationInterval, previousState, bodyKinematics, previousLatitude, config);

            final var newState44 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, propagationInterval,
                    previousState, bodyKinematics, previousLatitude, config, newState44);

            final var newState45 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    propagationInterval, previousState, fx, fy, fz, config);

            final var newState46 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, propagationInterval,
                    previousState, fx, fy, fz, config, newState46);

            final var newState47 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    propagationInterval, previousState, fx, fy, fz, previousLatitude, config);

            final var newState48 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, propagationInterval,
                    previousState, fx, fy, fz, previousLatitude, config, newState48);

            final var positionAndVelocity = new ECEFPositionAndVelocity(userEcefPosition, userEcefVelocity);
            final var newState49 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, config);

            final var newState50 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, config, newState50);

            final var newState51 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitude, config);

            final var newState52 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, previousLatitude, config, newState52);

            final var newState53 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, config);

            final var newState54 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, TIME_INTERVAL_SECONDS, previousState,
                    fx, fy, fz, config, newState54);

            final var newState55 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitude, config);

            final var newState56 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, TIME_INTERVAL_SECONDS, previousState,
                    fx, fy, fz, previousLatitude, config, newState56);

            final var newState57 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    propagationInterval, previousState, bodyKinematics, config);

            final var newState58 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, propagationInterval, previousState,
                    bodyKinematics, config, newState58);

            final var newState59 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    propagationInterval, previousState, bodyKinematics, previousLatitude, config);

            final var newState60 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, propagationInterval, previousState,
                    bodyKinematics, previousLatitude, config, newState60);

            final var newState61 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    propagationInterval, previousState, fx, fy, fz, config);

            final var newState62 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, propagationInterval, previousState,
                    fx, fy, fz, config, newState62);

            final var newState63 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    propagationInterval, previousState, fx, fy, fz, previousLatitude, config);

            final var newState64 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, propagationInterval, previousState,
                    fx, fy, fz, previousLatitude, config, newState64);

            final var previousLatitudeAngle = new Angle(previousLatitude, AngleUnit.RADIANS);
            final var newState65 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState66 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, previousLatitudeAngle, config, newState66);

            final var newState67 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var newState68 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, previousLatitudeAngle, config, newState68);

            final var newState69 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState70 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, previousLatitudeAngle, config, newState70);

            final var newState71 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var newState72 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, TIME_INTERVAL_SECONDS, previousState, 
                    fx, fy, fz, previousLatitudeAngle, config, newState72);

            final var newState73 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    propagationInterval, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState74 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState75 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, propagationInterval,
                    previousState, bodyKinematics, previousLatitudeAngle, config, newState75);

            final var newState76 = INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity,
                    propagationInterval, previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var newState77 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userEcefPosition, userEcefVelocity, propagationInterval,
                    previousState, fx, fy, fz, previousLatitudeAngle, config, newState77);

            final var newState78 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    propagationInterval, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState79 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                    bodyKinematics, previousLatitudeAngle, config, newState79);

            final var newState80 = INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, 
                    propagationInterval, previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var newState81 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                    fx, fy, fz, previousLatitudeAngle, config, newState81);

            final var newState82 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState83 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, previousLatitudeAngle, config, newState83);

            final var newState84 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var newState85 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, previousLatitudeAngle, config, newState85);

            final var newState86 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    propagationInterval, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState87 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState88 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, propagationInterval,
                    previousState, bodyKinematics, previousLatitudeAngle, config, newState88);

            final var newState89 = INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, 
                    propagationInterval, previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var newState90 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(userPosition, userEcefVelocity, propagationInterval,
                    previousState, fx, fy, fz, previousLatitudeAngle, config, newState90);

            final var newState91 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState92 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, previousLatitudeAngle, config, newState92);

            final var newState93 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var newState94 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, TIME_INTERVAL_SECONDS, previousState,
                    fx, fy, fz, previousLatitudeAngle, config, newState94);

            final var newState95 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    propagationInterval, previousState, bodyKinematics, previousLatitudeAngle, config);

            final var newState96 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, propagationInterval, previousState,
                    bodyKinematics, previousLatitudeAngle, config, newState96);

            final var newState97 = INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, 
                    propagationInterval, previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var newState98 = new INSLooselyCoupledKalmanState();
            INSLooselyCoupledKalmanEpochEstimator.estimate(positionAndVelocity, propagationInterval, previousState,
                    fx, fy, fz, previousLatitudeAngle, config, newState98);

            assertEquals(newState1, newState2);
            assertEquals(newState1, newState3);
            assertEquals(newState1, newState4);
            assertEquals(newState1, newState5);
            assertEquals(newState1, newState6);
            assertEquals(newState1, newState7);
            assertEquals(newState1, newState8);
            assertEquals(newState1, newState9);
            assertEquals(newState1, newState10);
            assertEquals(newState1, newState11);
            assertEquals(newState1, newState12);
            assertEquals(newState1, newState13);
            assertEquals(newState1, newState14);
            assertEquals(newState1, newState15);
            assertEquals(newState1, newState16);
            assertEquals(newState1, newState17);
            assertEquals(newState1, newState18);
            assertEquals(newState1, newState19);
            assertEquals(newState1, newState20);
            assertEquals(newState1, newState21);
            assertEquals(newState1, newState22);
            assertEquals(newState1, newState23);
            assertEquals(newState1, newState24);
            assertEquals(newState1, newState25);
            assertEquals(newState1, newState26);
            assertEquals(newState1, newState27);
            assertEquals(newState1, newState28);
            assertEquals(newState1, newState29);
            assertEquals(newState1, newState30);
            assertEquals(newState1, newState31);
            assertEquals(newState1, newState32);
            assertEquals(newState1, newState33);
            assertEquals(newState1, newState34);
            assertEquals(newState1, newState35);
            assertEquals(newState1, newState36);
            assertEquals(newState1, newState37);
            assertEquals(newState1, newState38);
            assertEquals(newState1, newState39);
            assertEquals(newState1, newState40);
            assertEquals(newState1, newState41);
            assertEquals(newState1, newState42);
            assertEquals(newState1, newState43);
            assertEquals(newState1, newState44);
            assertEquals(newState1, newState45);
            assertEquals(newState1, newState46);
            assertEquals(newState1, newState47);
            assertEquals(newState1, newState48);
            assertEquals(newState1, newState49);
            assertEquals(newState1, newState50);
            assertEquals(newState1, newState51);
            assertEquals(newState1, newState52);
            assertEquals(newState1, newState53);
            assertEquals(newState1, newState54);
            assertEquals(newState1, newState55);
            assertEquals(newState1, newState56);
            assertEquals(newState1, newState57);
            assertEquals(newState1, newState58);
            assertEquals(newState1, newState59);
            assertEquals(newState1, newState60);
            assertEquals(newState1, newState61);
            assertEquals(newState1, newState62);
            assertEquals(newState1, newState63);
            assertEquals(newState1, newState64);
            assertEquals(newState1, newState65);
            assertEquals(newState1, newState66);
            assertEquals(newState1, newState67);
            assertEquals(newState1, newState68);
            assertEquals(newState1, newState69);
            assertEquals(newState1, newState70);
            assertEquals(newState1, newState71);
            assertEquals(newState1, newState72);
            assertEquals(newState1, newState73);
            assertEquals(newState1, newState74);
            assertEquals(newState1, newState75);
            assertEquals(newState1, newState76);
            assertEquals(newState1, newState77);
            assertEquals(newState1, newState78);
            assertEquals(newState1, newState79);
            assertEquals(newState1, newState80);
            assertEquals(newState1, newState81);
            assertEquals(newState1, newState82);
            assertEquals(newState1, newState83);
            assertEquals(newState1, newState84);
            assertEquals(newState1, newState85);
            assertEquals(newState1, newState86);
            assertEquals(newState1, newState87);
            assertEquals(newState1, newState88);
            assertEquals(newState1, newState89);
            assertEquals(newState1, newState90);
            assertEquals(newState1, newState91);
            assertEquals(newState1, newState92);
            assertEquals(newState1, newState93);
            assertEquals(newState1, newState94);
            assertEquals(newState1, newState95);
            assertEquals(newState1, newState96);
            assertEquals(newState1, newState97);
            assertEquals(newState1, newState98);

            final var newState = estimate(x, y, z, vx, vy, vz, previousState, fx, fy, fz, previousLatitude, config);

            if (!newState.equals(newState1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(newState.equals(newState1, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {

        final var omegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, Constants.EARTH_ROTATION_RATE});

        // SYSTEM PROPAGATION PHASE

        // 1. Determine transition matrix using (14.50) (first-order approx)
        final var phiMatrix = Matrix.identity(15, 15);
        phiMatrix.setSubmatrix(0, 0, 2, 2,
                phiMatrix.getSubmatrix(0, 0, 2, 2)
                        .subtractAndReturnNew(omegaIe.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS)));

        final var estCbeOld = previousState.getBodyToEcefCoordinateTransformationMatrix();
        phiMatrix.setSubmatrix(0, 12, 2, 14,
                estCbeOld.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        final var measFibb = Matrix.newFromArray(new double[]{fx, fy, fz});
        phiMatrix.setSubmatrix(3, 0, 5, 2,
                Utils.skewMatrix(estCbeOld.multiplyAndReturnNew(measFibb))
                        .multiplyByScalarAndReturnNew(-TIME_INTERVAL_SECONDS));

        phiMatrix.setSubmatrix(3, 3, 5, 5,
                phiMatrix.getSubmatrix(3, 3, 5, 5)
                        .subtractAndReturnNew(omegaIe.multiplyByScalarAndReturnNew(2.0 * TIME_INTERVAL_SECONDS)));

        final var sinPrevLat = Math.sin(previousLatitude);
        final var cosPrevLat = Math.cos(previousLatitude);
        final var sinPrevLat2 = sinPrevLat * sinPrevLat;
        final var cosPrevLat2 = cosPrevLat * cosPrevLat;

        final var geocentricRadius = Constants.EARTH_EQUATORIAL_RADIUS_WGS84
                / Math.sqrt(1.0 - Math.pow(Constants.EARTH_ECCENTRICITY * sinPrevLat, 2.0))
                * Math.sqrt(cosPrevLat2
                + Math.pow(1.0 - Constants.EARTH_ECCENTRICITY * Constants.EARTH_ECCENTRICITY, 2.0)
                * sinPrevLat2);

        final var prevX = previousState.getX();
        final var prevY = previousState.getY();
        final var prevZ = previousState.getZ();
        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(prevX, prevY, prevZ);
        final var g = gravity.asMatrix();

        final var estRebeOld = Matrix.newFromArray(new double[]{prevX, prevY, prevZ});

        final var gScaled = g.multiplyByScalarAndReturnNew(-2.0 * TIME_INTERVAL_SECONDS / geocentricRadius);
        final var estRebeOldTrans = estRebeOld.transposeAndReturnNew();
        final var previousPositionNorm = Math.sqrt(prevX * prevX + prevY * prevY + prevZ * prevZ);
        final var estRebeOldTransScaled = estRebeOldTrans.multiplyByScalarAndReturnNew(1.0 / previousPositionNorm);
        phiMatrix.setSubmatrix(3, 6, 5, 8,
                gScaled.multiplyAndReturnNew(estRebeOldTransScaled));

        phiMatrix.setSubmatrix(3, 9, 5, 11,
                estCbeOld.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        phiMatrix.setSubmatrix(6, 3, 8, 5,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        // 2. Determine approximate system noise covariance matrix using (14.82)
        final var qPrimeMatrix = new Matrix(15, 15);
        qPrimeMatrix.setSubmatrix(0, 0, 2, 2,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getGyroNoisePSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(3, 3, 5, 5,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getAccelerometerNoisePSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(9, 9, 11, 11,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getAccelerometerBiasPSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(12, 12, 14, 14,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getGyroBiasPSD() * TIME_INTERVAL_SECONDS));

        // 3. Propagate state estimates using (3.14) noting that all states are zero
        final var xEstPropagated = new Matrix(15, 1);

        // 4. Propagate state estimation error covariance matrix using (3.46)
        final var pMatrixOld = previousState.getCovariance();
        final var phiTrans = phiMatrix.transposeAndReturnNew();
        final var halfQ = qPrimeMatrix.multiplyByScalarAndReturnNew(0.5);
        final var tmp1 = pMatrixOld.addAndReturnNew(halfQ);
        final var pMatrixPropagated = phiMatrix.multiplyAndReturnNew(tmp1);
        pMatrixPropagated.multiply(phiTrans);
        pMatrixPropagated.add(halfQ);

        // MEASUREMENT UPDATE PHASE

        // 5. Set up a measurement matrix using (14.115)
        final var hMatrix = new Matrix(6, 15);
        hMatrix.setSubmatrix(0, 6, 2, 8,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0));
        hMatrix.setSubmatrix(3, 3, 5, 5,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0));

        // 6. Set up a measurement noise covariance matrix assuming all components of
        // GNSS position and velocity are independent and have equal variance
        final var rMatrix = new Matrix(6, 6);
        rMatrix.setSubmatrix(0, 0, 2, 2,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        Math.pow(config.getPositionNoiseSD(), 2.0)));
        rMatrix.setSubmatrix(0, 3, 2, 5,
                new Matrix(3, 3));
        rMatrix.setSubmatrix(3, 0, 5, 2,
                new Matrix(3, 3));
        rMatrix.setSubmatrix(3, 3, 5, 5,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        Math.pow(config.getVelocityNoiseSD(), 2.0)));

        // 7. Calculate Kalman gain using (3.21)
        final var tmp2 = hMatrix.multiplyAndReturnNew(pMatrixPropagated).multiplyAndReturnNew(
                hMatrix.transposeAndReturnNew());
        tmp2.add(rMatrix);
        final var kMatrix = pMatrixPropagated.multiplyAndReturnNew(hMatrix.transposeAndReturnNew())
                .multiplyAndReturnNew(Utils.inverse(tmp2));

        // 8. Formulate measurement innovations using (14.102), noting that zero-lever arm is assumed here
        final var deltaZ = new Matrix(6, 1);

        final var prevVx = previousState.getVx();
        final var prevVy = previousState.getVy();
        final var prevVz = previousState.getVz();
        final var estVebeOld = Matrix.newFromArray(new double[]{prevVx, prevVy, prevVz});

        final var gnssRebe = Matrix.newFromArray(new double[]{x, y, z});
        final var gnssVebe = Matrix.newFromArray(new double[]{vx, vy, vz});

        deltaZ.setSubmatrix(0, 0, 2, 0,
                gnssRebe.subtractAndReturnNew(estRebeOld));
        deltaZ.setSubmatrix(3, 0, 5, 0,
                gnssVebe.subtractAndReturnNew(estVebeOld));

        // 9. Update state estimates using (3.24)
        final var xEstNew = xEstPropagated.addAndReturnNew(kMatrix.multiplyAndReturnNew(deltaZ));

        // 10. Update state estimation error covariance matrix using (3.25)
        final var pMatrixNew = (Matrix.identity(15, 15)
                .subtractAndReturnNew(kMatrix.multiplyAndReturnNew(hMatrix)))
                .multiplyAndReturnNew(pMatrixPropagated);

        // CLOSED-LOOP CORRECTION

        // Correct attitude, velocity, and position using (14.7-9)
        final var estCbeNew = (Matrix.identity(3, 3).subtractAndReturnNew(
                Utils.skewMatrix(
                        xEstNew.getSubmatrix(0, 0, 2, 0))))
                .multiplyAndReturnNew(estCbeOld);
        final var estVebeNew = estVebeOld.subtractAndReturnNew(
                xEstNew.getSubmatrix(3, 0, 5, 0));
        final var estRebeNew = estRebeOld.subtractAndReturnNew(
                xEstNew.getSubmatrix(6, 0, 8, 0));

        // Update IMU bias estimates
        final var estIMUbiasOld = Matrix.newFromArray(new double[]{
                previousState.getAccelerationBiasX(),
                previousState.getAccelerationBiasY(),
                previousState.getAccelerationBiasZ(),
                previousState.getGyroBiasX(),
                previousState.getGyroBiasY(),
                previousState.getGyroBiasZ()
        });
        final var estIMUbiasNew = estIMUbiasOld.addAndReturnNew(
                xEstNew.getSubmatrix(9, 0, 14, 0));

        return new INSLooselyCoupledKalmanState(estCbeNew,
                estVebeNew.getElementAtIndex(0),
                estVebeNew.getElementAtIndex(1),
                estVebeNew.getElementAtIndex(2),
                estRebeNew.getElementAtIndex(0),
                estRebeNew.getElementAtIndex(1),
                estRebeNew.getElementAtIndex(2),
                estIMUbiasNew.getElementAtIndex(0),
                estIMUbiasNew.getElementAtIndex(1),
                estIMUbiasNew.getElementAtIndex(2),
                estIMUbiasNew.getElementAtIndex(3),
                estIMUbiasNew.getElementAtIndex(4),
                estIMUbiasNew.getElementAtIndex(5),
                pMatrixNew);
    }
}
