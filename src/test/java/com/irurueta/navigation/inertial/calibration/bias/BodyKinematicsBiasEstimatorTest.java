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
package com.irurueta.navigation.inertial.calibration.bias;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class BodyKinematicsBiasEstimatorTest implements BodyKinematicsBiasEstimatorListener {

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final int N_SAMPLES = 100000;

    private static final double MIN_TIME_INTERVAL = 0.01;
    private static final double MAX_TIME_INTERVAL = 0.04;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;

    private static final double ACCELEROMETER_BIAS_ERROR = 1e-2;
    private static final double GYRO_BIAS_ERROR = 1e-4;

    private static final double ACCELEROMETER_NOISE_ROOT_PSD_ERROR = 1e-5;
    private static final double GYRO_NOISE_ROOT_PSD_ERROR = 1e-5;

    private static final int TIMES = 100;

    private int start = 0;
    private int bodyKinematicsAdded = 0;
    private int reset = 0;

    @Test
    void testConstructor1() throws WrongSizeException {

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor2() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        nedC.setEulerAngles(roll, pitch, yaw);
        nedFrame1.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame1, ecefFrame1);
        ecefFrame1.getCoordinateTransformation(ecefC);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedC);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new BodyKinematicsBiasEstimator(
                new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)));
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor6() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedPosition1, nedC);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefC(), ecefC);
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new BodyKinematicsBiasEstimator(
                nedPosition1, new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME)));
    }

    @Test
    void testConstructor7() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var kinematics1 = new BodyKinematics();
            final var kinematics2 = new BodyKinematics();
            final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final var time2 = new Time(0.0, TimeUnit.SECOND);
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
            final var nedFrame2 = new NEDFrame();
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
            final var ecefFrame2 = new ECEFFrame();
            final var nedPosition1 = nedFrame1.getPosition();
            final var nedPosition2 = new NEDPosition();
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefPosition2 = new ECEFPosition();
            final var ecefC = ecefFrame1.getCoordinateTransformation();
            final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
            final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

            final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, ecefPosition1);

            // test constructor
            final var estimator = new BodyKinematicsBiasEstimator(ecefPosition1, nedC);

            // check default values
            assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                    0.0);
            assertEquals(time1, estimator.getTimeIntervalAsTime());
            estimator.getTimeIntervalAsTime(time2);
            assertEquals(time1, time2);
            if (!estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR));
            estimator.getEcefPosition(ecefPosition2);
            assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));
            if (!estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR));
            estimator.getEcefFrame(ecefFrame2);
            assertTrue(ecefFrame2.equals(ecefFrame1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
            estimator.getNedFrame(nedFrame2);
            assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            estimator.getNedPosition(nedPosition2);
            assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC, ABSOLUTE_ERROR));
            estimator.getEcefC(c);
            assertTrue(ecefC.equals(c, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
            estimator.getNedC(c);
            assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
            assertNull(estimator.getListener());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertEquals(0.0, estimator.getBiasFx(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
            estimator.getBiasFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFy(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
            estimator.getBiasFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFz(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
            estimator.getBiasFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
            estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
            estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
            estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var aTriad1 = estimator.getBiasF();
            assertEquals(0.0, aTriad1.getValueX(), 0.0);
            assertEquals(0.0, aTriad1.getValueY(), 0.0);
            assertEquals(0.0, aTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
            final var aTriad2 = new AccelerationTriad();
            estimator.getBiasF(aTriad2);
            assertEquals(aTriad1, aTriad2);
            final var wTriad1 = estimator.getBiasAngularRate();
            assertEquals(0.0, wTriad1.getValueX(), 0.0);
            assertEquals(0.0, wTriad1.getValueY(), 0.0);
            assertEquals(0.0, wTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
            final var wTriad2 = new AngularSpeedTriad();
            estimator.getBiasAngularRate(wTriad2);
            assertEquals(wTriad1, wTriad2);
            assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
            estimator.getBiasesAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getVarianceFx(), 0.0);
            assertEquals(0.0, estimator.getVarianceFy(), 0.0);
            assertEquals(0.0, estimator.getVarianceFz(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
            estimator.getStandardDeviationFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
            estimator.getStandardDeviationFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
            estimator.getStandardDeviationFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            final var aStdTriad1 = estimator.getStandardDeviationF();
            assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
            final var aStdTriad2 = new AccelerationTriad();
            estimator.getStandardDeviationF(aStdTriad2);
            assertEquals(aStdTriad1, aStdTriad2);
            assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
            assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
            estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
            estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
            estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
            estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
            assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
            final var wStdTriad2 = new AngularSpeedTriad();
            estimator.getStandardDeviationAngularRate(wStdTriad2);
            assertEquals(wStdTriad1, wStdTriad2);
            assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
            assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
            estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
            estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getPSDFx(), 0.0);
            assertEquals(0.0, estimator.getPSDFy(), 0.0);
            assertEquals(0.0, estimator.getPSDFz(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
            assertEquals(m1, estimator.getAccelerometerBias());
            estimator.getAccelerometerBias(m2);
            assertEquals(m1, m2);
            assertEquals(m1, estimator.getGyroBias());
            estimator.getGyroBias(m2);
            assertEquals(m1, m2);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
            final var elapsedTime1 = estimator.getElapsedTime();
            assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
            final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
            estimator.getElapsedTime(elapsedTime2);
            assertEquals(elapsedTime1, elapsedTime2);
            assertFalse(estimator.isRunning());
            assertTrue(estimator.getExpectedKinematics().equals(expectedKinematics, ABSOLUTE_ERROR));
            estimator.getExpectedKinematics(kinematics2);
            assertTrue(expectedKinematics.equals(kinematics2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> new BodyKinematicsBiasEstimator(ecefPosition1, new CoordinateTransformation(
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME)));
            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testConstructor8() throws WrongSizeException {

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(this);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor9() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        nedC.setEulerAngles(roll, pitch, yaw);
        nedFrame1.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame1, ecefFrame1);
        ecefFrame1.getCoordinateTransformation(ecefC);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedC, this);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME), this));
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, this);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, this);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height,
                this);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor13() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedPosition1, nedC, this);

        // check default values
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(nedPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), this));
    }

    @Test
    void testConstructor14() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var kinematics1 = new BodyKinematics();
            final var kinematics2 = new BodyKinematics();
            final var time1 = new Time(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final var time2 = new Time(0.0, TimeUnit.SECOND);
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
            final var nedFrame2 = new NEDFrame();
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
            final var ecefFrame2 = new ECEFFrame();
            final var nedPosition1 = nedFrame1.getPosition();
            final var nedPosition2 = new NEDPosition();
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefPosition2 = new ECEFPosition();
            final var ecefC = ecefFrame1.getCoordinateTransformation();
            final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
            final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

            final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, ecefC, ecefC,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

            // test constructor
            final var estimator = new BodyKinematicsBiasEstimator(ecefPosition1, nedC, this);

            // check default values
            assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 
                    0.0);
            assertEquals(time1, estimator.getTimeIntervalAsTime());
            estimator.getTimeIntervalAsTime(time2);
            assertEquals(time1, time2);
            if (!estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR));
            estimator.getEcefPosition(ecefPosition2);
            assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));
            if (!estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR));
            estimator.getEcefFrame(ecefFrame2);
            assertTrue(ecefFrame2.equals(ecefFrame1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
            estimator.getNedFrame(nedFrame2);
            assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            estimator.getNedPosition(nedPosition2);
            assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC, ABSOLUTE_ERROR));
            estimator.getEcefC(c);
            assertTrue(ecefC.equals(c, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
            estimator.getNedC(c);
            assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
            assertSame(this, estimator.getListener());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertEquals(0.0, estimator.getBiasFx(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
            estimator.getBiasFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFy(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
            estimator.getBiasFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFz(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
            estimator.getBiasFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
            estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
            estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
            estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var aTriad1 = estimator.getBiasF();
            assertEquals(0.0, aTriad1.getValueX(), 0.0);
            assertEquals(0.0, aTriad1.getValueY(), 0.0);
            assertEquals(0.0, aTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
            final var aTriad2 = new AccelerationTriad();
            estimator.getBiasF(aTriad2);
            assertEquals(aTriad1, aTriad2);
            final var wTriad1 = estimator.getBiasAngularRate();
            assertEquals(0.0, wTriad1.getValueX(), 0.0);
            assertEquals(0.0, wTriad1.getValueY(), 0.0);
            assertEquals(0.0, wTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
            final var wTriad2 = new AngularSpeedTriad();
            estimator.getBiasAngularRate(wTriad2);
            assertEquals(wTriad1, wTriad2);
            assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
            estimator.getBiasesAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getVarianceFx(), 0.0);
            assertEquals(0.0, estimator.getVarianceFy(), 0.0);
            assertEquals(0.0, estimator.getVarianceFz(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
            estimator.getStandardDeviationFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
            estimator.getStandardDeviationFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
            estimator.getStandardDeviationFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            final var aStdTriad1 = estimator.getStandardDeviationF();
            assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
            final var aStdTriad2 = new AccelerationTriad();
            estimator.getStandardDeviationF(aStdTriad2);
            assertEquals(aStdTriad1, aStdTriad2);
            assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
            assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
            estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
            estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
            estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
            assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
            estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
            assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
            final var wStdTriad2 = new AngularSpeedTriad();
            estimator.getStandardDeviationAngularRate(wStdTriad2);
            assertEquals(wStdTriad1, wStdTriad2);
            assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
            assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
            estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
            estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getPSDFx(), 0.0);
            assertEquals(0.0, estimator.getPSDFy(), 0.0);
            assertEquals(0.0, estimator.getPSDFz(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
            assertEquals(m1, estimator.getAccelerometerBias());
            estimator.getAccelerometerBias(m2);
            assertEquals(m1, m2);
            assertEquals(m1, estimator.getGyroBias());
            estimator.getGyroBias(m2);
            assertEquals(m1, m2);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
            final var elapsedTime1 = estimator.getElapsedTime();
            assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
            final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
            estimator.getElapsedTime(elapsedTime2);
            assertEquals(elapsedTime1, elapsedTime2);
            assertFalse(estimator.isRunning());
            assertTrue(estimator.getExpectedKinematics().equals(expectedKinematics, ABSOLUTE_ERROR));
            estimator.getExpectedKinematics(kinematics2);
            assertTrue(expectedKinematics.equals(kinematics2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> new BodyKinematicsBiasEstimator(ecefPosition1, new CoordinateTransformation(
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME)));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testConstructor15() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(timeInterval);

        // check default values
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(-1.0));
    }

    @Test
    void testConstructor16() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        nedC.setEulerAngles(roll, pitch, yaw);
        nedFrame1.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame1, ecefFrame1);
        ecefFrame1.getCoordinateTransformation(ecefC);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedC, timeInterval);

        // check default values
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME), timeInterval));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(nedC, -1.0));
    }

    @Test
    void testConstructor17() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval);

        // check default values
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0));
    }

    @Test
    void testConstructor18() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval);

        // check default values
        assertEquals(estimator.getTimeInterval(), timeInterval, 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0));
    }

    @Test
    void testConstructor19() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval);

        // check default values
        assertEquals(estimator.getTimeInterval(), timeInterval, 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0));
    }

    @Test
    void testConstructor20() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedPosition1, nedC, timeInterval);

        // check default values
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(nedPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), timeInterval));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(nedPosition1, nedC,
                -1.0));
    }

    @Test
    void testConstructor21() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var kinematics1 = new BodyKinematics();
            final var kinematics2 = new BodyKinematics();
            final var time1 = new Time(timeInterval, TimeUnit.SECOND);
            final var time2 = new Time(0.0, TimeUnit.SECOND);
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
            final var nedFrame2 = new NEDFrame();
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
            final var ecefFrame2 = new ECEFFrame();
            final var nedPosition1 = nedFrame1.getPosition();
            final var nedPosition2 = new NEDPosition();
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefPosition2 = new ECEFPosition();
            final var ecefC = ecefFrame1.getCoordinateTransformation();
            final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
            final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

            final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC,
                    ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

            // test constructor
            final var estimator = new BodyKinematicsBiasEstimator(ecefPosition1, nedC, timeInterval);

            // check default values
            assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
            assertEquals(time1, estimator.getTimeIntervalAsTime());
            estimator.getTimeIntervalAsTime(time2);
            assertEquals(time1, time2);
            if (!estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR));
            estimator.getEcefPosition(ecefPosition2);
            assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));
            if (!estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR));
            estimator.getEcefFrame(ecefFrame2);
            assertTrue(ecefFrame2.equals(ecefFrame1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
            estimator.getNedFrame(nedFrame2);
            assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            estimator.getNedPosition(nedPosition2);
            assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC, ABSOLUTE_ERROR));
            estimator.getEcefC(c);
            assertTrue(ecefC.equals(c, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
            estimator.getNedC(c);
            assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
            assertNull(estimator.getListener());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertEquals(0.0, estimator.getBiasFx(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
            estimator.getBiasFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFy(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
            estimator.getBiasFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFz(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
            estimator.getBiasFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
            estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
            estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
            estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var aTriad1 = estimator.getBiasF();
            assertEquals(0.0, aTriad1.getValueX(), 0.0);
            assertEquals(0.0, aTriad1.getValueY(), 0.0);
            assertEquals(0.0, aTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
            final var aTriad2 = new AccelerationTriad();
            estimator.getBiasF(aTriad2);
            assertEquals(aTriad1, aTriad2);
            final var wTriad1 = estimator.getBiasAngularRate();
            assertEquals(0.0, wTriad1.getValueX(), 0.0);
            assertEquals(0.0, wTriad1.getValueY(), 0.0);
            assertEquals(0.0, wTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
            final var wTriad2 = new AngularSpeedTriad();
            estimator.getBiasAngularRate(wTriad2);
            assertEquals(wTriad1, wTriad2);
            assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
            estimator.getBiasesAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getVarianceFx(), 0.0);
            assertEquals(0.0, estimator.getVarianceFy(), 0.0);
            assertEquals(0.0, estimator.getVarianceFz(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
            estimator.getStandardDeviationFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
            estimator.getStandardDeviationFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
            estimator.getStandardDeviationFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            final var aStdTriad1 = estimator.getStandardDeviationF();
            assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
            final var aStdTriad2 = new AccelerationTriad();
            estimator.getStandardDeviationF(aStdTriad2);
            assertEquals(aStdTriad1, aStdTriad2);
            assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
            assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
            estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
            assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
            estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
            assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
            estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
            assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
            estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
            assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
            final var wStdTriad2 = new AngularSpeedTriad();
            estimator.getStandardDeviationAngularRate(wStdTriad2);
            assertEquals(wStdTriad1, wStdTriad2);
            assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
            assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
            estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
            estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getPSDFx(), 0.0);
            assertEquals(0.0, estimator.getPSDFy(), 0.0);
            assertEquals(0.0, estimator.getPSDFz(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
            assertEquals(m1, estimator.getAccelerometerBias());
            estimator.getAccelerometerBias(m2);
            assertEquals(m1, m2);
            assertEquals(m1, estimator.getGyroBias());
            estimator.getGyroBias(m2);
            assertEquals(m1, m2);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
            final var elapsedTime1 = estimator.getElapsedTime();
            assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
            final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
            estimator.getElapsedTime(elapsedTime2);
            assertEquals(elapsedTime1, elapsedTime2);
            assertFalse(estimator.isRunning());
            assertTrue(estimator.getExpectedKinematics().equals(expectedKinematics, ABSOLUTE_ERROR));
            estimator.getExpectedKinematics(kinematics2);
            assertTrue(expectedKinematics.equals(kinematics2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> new BodyKinematicsBiasEstimator(ecefPosition1, new CoordinateTransformation(
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME),
                            timeInterval));

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(ecefPosition1, nedC,
                    -1.0));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testConstructor22() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(timeInterval, this);

        // check default values
        assertEquals(estimator.getTimeInterval(), timeInterval, 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(-1.0));
    }

    @Test
    void testConstructor23() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        nedC.setEulerAngles(roll, pitch, yaw);
        nedFrame1.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame1, ecefFrame1);
        ecefFrame1.getCoordinateTransformation(ecefC);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedC, timeInterval, this);

        // check default values
        assertEquals(estimator.getTimeInterval(), timeInterval, 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME), timeInterval));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(nedC, -1.0));
    }

    @Test
    void testConstructor24() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval, this);

        // check default values
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor25() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval, this);

        // check default values
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0, this));
    }

    @Test
    void testConstructor26() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval, this);

        // check default values
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0, this));
    }

    @Test
    void testConstructor27() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval, TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedPosition1, nedC, timeInterval, this);

        // check default values
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(nedPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), timeInterval, this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(nedPosition1, nedC,
                -1.0, this));
    }

    @Test
    void testConstructor28() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var kinematics1 = new BodyKinematics();
            final var kinematics2 = new BodyKinematics();
            final var time1 = new Time(timeInterval, TimeUnit.SECOND);
            final var time2 = new Time(0.0, TimeUnit.SECOND);
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
            final var nedFrame2 = new NEDFrame();
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
            final var ecefFrame2 = new ECEFFrame();
            final var nedPosition1 = nedFrame1.getPosition();
            final var nedPosition2 = new NEDPosition();
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefPosition2 = new ECEFPosition();
            final var ecefC = ecefFrame1.getCoordinateTransformation();
            final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
            final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

            final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC,
                    ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

            // test constructor
            final var estimator = new BodyKinematicsBiasEstimator(ecefPosition1, nedC, timeInterval, this);

            // check default values
            assertEquals(estimator.getTimeInterval(), timeInterval, 0.0);
            assertEquals(time1, estimator.getTimeIntervalAsTime());
            estimator.getTimeIntervalAsTime(time2);
            assertEquals(time1, time2);
            if (!estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR));
            estimator.getEcefPosition(ecefPosition2);
            assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));
            if (!estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR));
            estimator.getEcefFrame(ecefFrame2);
            assertTrue(ecefFrame2.equals(ecefFrame1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
            estimator.getNedFrame(nedFrame2);
            assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            estimator.getNedPosition(nedPosition2);
            assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC, ABSOLUTE_ERROR));
            estimator.getEcefC(c);
            assertTrue(ecefC.equals(c, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
            estimator.getNedC(c);
            assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
            assertSame(this, estimator.getListener());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertEquals(0.0, estimator.getBiasFx(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
            estimator.getBiasFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFy(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
            estimator.getBiasFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFz(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
            estimator.getBiasFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
            estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
            estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
            estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var aTriad1 = estimator.getBiasF();
            assertEquals(0.0, aTriad1.getValueX(), 0.0);
            assertEquals(0.0, aTriad1.getValueY(), 0.0);
            assertEquals(0.0, aTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
            final var aTriad2 = new AccelerationTriad();
            estimator.getBiasF(aTriad2);
            assertEquals(aTriad1, aTriad2);
            final var wTriad1 = estimator.getBiasAngularRate();
            assertEquals(0.0, wTriad1.getValueX(), 0.0);
            assertEquals(0.0, wTriad1.getValueY(), 0.0);
            assertEquals(0.0, wTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
            final var wTriad2 = new AngularSpeedTriad();
            estimator.getBiasAngularRate(wTriad2);
            assertEquals(wTriad1, wTriad2);
            assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
            estimator.getBiasesAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getVarianceFx(), 0.0);
            assertEquals(0.0, estimator.getVarianceFy(), 0.0);
            assertEquals(0.0, estimator.getVarianceFz(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
            estimator.getStandardDeviationFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
            estimator.getStandardDeviationFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
            estimator.getStandardDeviationFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            final var aStdTriad1 = estimator.getStandardDeviationF();
            assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
            final var aStdTriad2 = new AccelerationTriad();
            estimator.getStandardDeviationF(aStdTriad2);
            assertEquals(aStdTriad1, aStdTriad2);
            assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
            assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
            estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
            estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
            estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
            assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(), angularSpeed1);
            estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
            assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
            final var wStdTriad2 = new AngularSpeedTriad();
            estimator.getStandardDeviationAngularRate(wStdTriad2);
            assertEquals(wStdTriad1, wStdTriad2);
            assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
            assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
            estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
            estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getPSDFx(), 0.0);
            assertEquals(0.0, estimator.getPSDFy(), 0.0);
            assertEquals(0.0, estimator.getPSDFz(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
            assertEquals(m1, estimator.getAccelerometerBias());
            estimator.getAccelerometerBias(m2);
            assertEquals(m1, m2);
            assertEquals(m1, estimator.getGyroBias());
            estimator.getGyroBias(m2);
            assertEquals(m1, m2);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
            final var elapsedTime1 = estimator.getElapsedTime();
            assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
            final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
            estimator.getElapsedTime(elapsedTime2);
            assertEquals(elapsedTime1, elapsedTime2);
            assertFalse(estimator.isRunning());
            assertTrue(estimator.getExpectedKinematics().equals(expectedKinematics, ABSOLUTE_ERROR));
            estimator.getExpectedKinematics(kinematics2);
            assertTrue(expectedKinematics.equals(kinematics2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> new BodyKinematicsBiasEstimator(ecefPosition1, new CoordinateTransformation(
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME),
                            timeInterval, this));

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(ecefPosition1, nedC,
                    -1.0, this));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testConstructor29() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(timeInterval);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(-1.0));
    }

    @Test
    void testConstructor30() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        nedC.setEulerAngles(roll, pitch, yaw);
        nedFrame1.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame1, ecefFrame1);
        ecefFrame1.getCoordinateTransformation(ecefC);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedC, timeInterval);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME), timeInterval));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(nedC, -1.0));
    }

    @Test
    void testConstructor31() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0));
    }

    @Test
    void testConstructor32() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0));
    }

    @Test
    void testConstructor33() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0));
    }

    @Test
    void testConstructor34() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedPosition1, nedC, timeInterval);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(estimator.getBiasFzAsAcceleration(), acceleration1);
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(), angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(nedPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), timeInterval));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(nedPosition1, nedC, -1.0));
    }

    @Test
    void testConstructor35() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), 
                    TimeUnit.SECOND);

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var kinematics1 = new BodyKinematics();
            final var kinematics2 = new BodyKinematics();
            final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
            final var time2 = new Time(0.0, TimeUnit.SECOND);
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
            final var nedFrame2 = new NEDFrame();
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
            final var ecefFrame2 = new ECEFFrame();
            final var nedPosition1 = nedFrame1.getPosition();
            final var nedPosition2 = new NEDPosition();
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefPosition2 = new ECEFPosition();
            final var ecefC = ecefFrame1.getCoordinateTransformation();
            final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
            final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

            final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC,
                    ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

            // test constructor
            final var estimator = new BodyKinematicsBiasEstimator(ecefPosition1, nedC, timeInterval);

            // check default values
            assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
            assertEquals(time1, estimator.getTimeIntervalAsTime());
            estimator.getTimeIntervalAsTime(time2);
            assertEquals(time1, time2);
            if (!estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR));
            estimator.getEcefPosition(ecefPosition2);
            assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));
            if (!estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR));
            estimator.getEcefFrame(ecefFrame2);
            assertTrue(ecefFrame2.equals(ecefFrame1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
            estimator.getNedFrame(nedFrame2);
            assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            estimator.getNedPosition(nedPosition2);
            assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC, ABSOLUTE_ERROR));
            estimator.getEcefC(c);
            assertTrue(ecefC.equals(c, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
            estimator.getNedC(c);
            assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
            assertNull(estimator.getListener());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertEquals(0.0, estimator.getBiasFx(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
            estimator.getBiasFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFy(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
            estimator.getBiasFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFz(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
            estimator.getBiasFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
            estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
            estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
            estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var aTriad1 = estimator.getBiasF();
            assertEquals(0.0, aTriad1.getValueX(), 0.0);
            assertEquals(0.0, aTriad1.getValueY(), 0.0);
            assertEquals(0.0, aTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
            final var aTriad2 = new AccelerationTriad();
            estimator.getBiasF(aTriad2);
            assertEquals(aTriad1, aTriad2);
            final var wTriad1 = estimator.getBiasAngularRate();
            assertEquals(0.0, wTriad1.getValueX(), 0.0);
            assertEquals(0.0, wTriad1.getValueY(), 0.0);
            assertEquals(0.0, wTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
            final var wTriad2 = new AngularSpeedTriad();
            estimator.getBiasAngularRate(wTriad2);
            assertEquals(wTriad1, wTriad2);
            assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
            estimator.getBiasesAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getVarianceFx(), 0.0);
            assertEquals(0.0, estimator.getVarianceFy(), 0.0);
            assertEquals(0.0, estimator.getVarianceFz(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
            estimator.getStandardDeviationFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
            estimator.getStandardDeviationFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
            estimator.getStandardDeviationFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            final var aStdTriad1 = estimator.getStandardDeviationF();
            assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
            final var aStdTriad2 = new AccelerationTriad();
            estimator.getStandardDeviationF(aStdTriad2);
            assertEquals(aStdTriad1, aStdTriad2);
            assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
            assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
            estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
            estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
            estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
            estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
            assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
            final var wStdTriad2 = new AngularSpeedTriad();
            estimator.getStandardDeviationAngularRate(wStdTriad2);
            assertEquals(wStdTriad1, wStdTriad2);
            assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
            assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
            estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
            estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getPSDFx(), 0.0);
            assertEquals(0.0, estimator.getPSDFy(), 0.0);
            assertEquals(0.0, estimator.getPSDFz(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
            assertEquals(m1, estimator.getAccelerometerBias());
            estimator.getAccelerometerBias(m2);
            assertEquals(m1, m2);
            assertEquals(m1, estimator.getGyroBias());
            estimator.getGyroBias(m2);
            assertEquals(m1, m2);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
            final var elapsedTime1 = estimator.getElapsedTime();
            assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
            final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
            estimator.getElapsedTime(elapsedTime2);
            assertEquals(elapsedTime1, elapsedTime2);
            assertFalse(estimator.isRunning());
            assertTrue(estimator.getExpectedKinematics().equals(expectedKinematics, ABSOLUTE_ERROR));
            estimator.getExpectedKinematics(kinematics2);
            assertTrue(expectedKinematics.equals(kinematics2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> new BodyKinematicsBiasEstimator(ecefPosition1, new CoordinateTransformation(
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME),
                            timeInterval));

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(ecefPosition1, nedC,
                    -1.0));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testConstructor36() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(timeInterval, this);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(estimator.getEcefC(), ecefC);
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(-1.0));
    }

    @Test
    void testConstructor37() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame();
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        nedC.setEulerAngles(roll, pitch, yaw);
        nedFrame1.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame1, ecefFrame1);
        ecefFrame1.getCoordinateTransformation(ecefC);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedC, timeInterval, this);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertEquals(nedFrame1, estimator.getNedFrame());
        estimator.getNedFrame(nedFrame2);
        assertEquals(nedFrame1, nedFrame2);
        assertEquals(nedPosition1, estimator.getNedPosition());
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertEquals(nedC, estimator.getNedC());
        estimator.getNedC(c);
        assertEquals(nedC, c);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(), acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME), timeInterval));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(nedC, -1.0));
    }

    @Test
    void testConstructor38() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval, this);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);
    }

    @Test
    void testConstructor39() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval, this);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0, this));
    }

    @Test
    void testConstructor40() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var nedC = nedFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(latitude, longitude, height, timeInterval, this);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(latitude, longitude, height,
                -1.0, this));
    }

    @Test
    void testConstructor41() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), TimeUnit.SECOND);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var kinematics1 = new BodyKinematics();
        final var kinematics2 = new BodyKinematics();
        final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
        final var time2 = new Time(0.0, TimeUnit.SECOND);
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);
        final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
        final var nedFrame2 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = new ECEFFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedPosition2 = new NEDPosition();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefPosition2 = new ECEFPosition();
        final var ecefC = ecefFrame1.getCoordinateTransformation();
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, 
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

        // test constructor
        final var estimator = new BodyKinematicsBiasEstimator(nedPosition1, nedC, timeInterval, this);

        // check default values
        assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
        assertEquals(time1, estimator.getTimeIntervalAsTime());
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);
        assertEquals(ecefFrame1, estimator.getEcefFrame());
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame2, ecefFrame1);
        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC, estimator.getEcefC());
        estimator.getEcefC(c);
        assertEquals(ecefC, c);
        assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
        estimator.getNedC(c);
        assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getBiasFx(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
        estimator.getBiasFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFy(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
        estimator.getBiasFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasFz(), 0.0);
        assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
        estimator.getBiasFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
        estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
        estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
        estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var aTriad1 = estimator.getBiasF();
        assertEquals(0.0, aTriad1.getValueX(), 0.0);
        assertEquals(0.0, aTriad1.getValueY(), 0.0);
        assertEquals(0.0, aTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
        final var aTriad2 = new AccelerationTriad();
        estimator.getBiasF(aTriad2);
        assertEquals(aTriad1, aTriad2);
        final var wTriad1 = estimator.getBiasAngularRate();
        assertEquals(0.0, wTriad1.getValueX(), 0.0);
        assertEquals(0.0, wTriad1.getValueY(), 0.0);
        assertEquals(0.0, wTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
        final var wTriad2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(wTriad2);
        assertEquals(wTriad1, wTriad2);
        assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
        estimator.getBiasesAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getVarianceFx(), 0.0);
        assertEquals(0.0, estimator.getVarianceFy(), 0.0);
        assertEquals(0.0, estimator.getVarianceFz(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
        assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var aStdTriad1 = estimator.getStandardDeviationF();
        assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
        final var aStdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationF(aStdTriad2);
        assertEquals(aStdTriad1, aStdTriad2);
        assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
        assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
        assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
        final var wStdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(wStdTriad2);
        assertEquals(wStdTriad1, wStdTriad2);
        assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
        assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(0.0, estimator.getPSDFx(), 0.0);
        assertEquals(0.0, estimator.getPSDFy(), 0.0);
        assertEquals(0.0, estimator.getPSDFz(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
        assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
        assertEquals(m1, estimator.getAccelerometerBias());
        estimator.getAccelerometerBias(m2);
        assertEquals(m1, m2);
        assertEquals(m1, estimator.getGyroBias());
        estimator.getGyroBias(m2);
        assertEquals(m1, m2);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertFalse(estimator.isRunning());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
        estimator.getExpectedKinematics(kinematics2);
        assertEquals(expectedKinematics, kinematics2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new BodyKinematicsBiasEstimator(nedPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME), timeInterval,
                        this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(nedPosition1, nedC,
                -1.0, this));
    }

    @Test
    void testConstructor42() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var timeInterval = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL), 
                    TimeUnit.SECOND);

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var acceleration2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
            final var angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            final var kinematics1 = new BodyKinematics();
            final var kinematics2 = new BodyKinematics();
            final var time1 = new Time(timeInterval.getValue(), TimeUnit.SECOND);
            final var time2 = new Time(0.0, TimeUnit.SECOND);
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var nedFrame1 = new NEDFrame(latitude, longitude, height, 0.0, 0.0, 0.0, nedC);
            final var nedFrame2 = new NEDFrame();
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
            final var ecefFrame2 = new ECEFFrame();
            final var nedPosition1 = nedFrame1.getPosition();
            final var nedPosition2 = new NEDPosition();
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefPosition2 = new ECEFPosition();
            final var ecefC = ecefFrame1.getCoordinateTransformation();
            final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
            final var m2 = new Matrix(BodyKinematics.COMPONENTS, 1);

            final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC,
                    ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition1);

            // test constructor
            final var estimator = new BodyKinematicsBiasEstimator(ecefPosition1, nedC, timeInterval, this);

            // check default values
            assertEquals(timeInterval.getValue().doubleValue(), estimator.getTimeInterval(), 0.0);
            assertEquals(time1, estimator.getTimeIntervalAsTime());
            estimator.getTimeIntervalAsTime(time2);
            assertEquals(time1, time2);
            if (!estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition1, ABSOLUTE_ERROR));
            estimator.getEcefPosition(ecefPosition2);
            assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));
            if (!estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefFrame().equals(ecefFrame1, ABSOLUTE_ERROR));
            estimator.getEcefFrame(ecefFrame2);
            assertTrue(ecefFrame2.equals(ecefFrame1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
            estimator.getNedFrame(nedFrame2);
            assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            estimator.getNedPosition(nedPosition2);
            assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC, ABSOLUTE_ERROR));
            estimator.getEcefC(c);
            assertTrue(ecefC.equals(c, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC, ABSOLUTE_ERROR));
            estimator.getNedC(c);
            assertTrue(nedC.equals(c, ABSOLUTE_ERROR));
            assertSame(this, estimator.getListener());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertEquals(0.0, estimator.getBiasFx(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFxAsAcceleration());
            estimator.getBiasFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFy(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFyAsAcceleration());
            estimator.getBiasFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasFz(), 0.0);
            assertEquals(acceleration1, estimator.getBiasFzAsAcceleration());
            estimator.getBiasFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getBiasAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateXAsAngularSpeed());
            estimator.getBiasAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateYAsAngularSpeed());
            estimator.getBiasAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getBiasAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getBiasAngularRateZAsAngularSpeed());
            estimator.getBiasAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var aTriad1 = estimator.getBiasF();
            assertEquals(0.0, aTriad1.getValueX(), 0.0);
            assertEquals(0.0, aTriad1.getValueY(), 0.0);
            assertEquals(0.0, aTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aTriad1.getUnit());
            final var aTriad2 = new AccelerationTriad();
            estimator.getBiasF(aTriad2);
            assertEquals(aTriad1, aTriad2);
            final var wTriad1 = estimator.getBiasAngularRate();
            assertEquals(0.0, wTriad1.getValueX(), 0.0);
            assertEquals(0.0, wTriad1.getValueY(), 0.0);
            assertEquals(0.0, wTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wTriad1.getUnit());
            final var wTriad2 = new AngularSpeedTriad();
            estimator.getBiasAngularRate(wTriad2);
            assertEquals(wTriad1, wTriad2);
            assertEquals(kinematics1, estimator.getBiasesAsBodyKinematics());
            estimator.getBiasesAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getVarianceFx(), 0.0);
            assertEquals(0.0, estimator.getVarianceFy(), 0.0);
            assertEquals(0.0, estimator.getVarianceFz(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getStandardDeviationFx(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFxAsAcceleration());
            estimator.getStandardDeviationFxAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFy(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFyAsAcceleration());
            estimator.getStandardDeviationFyAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationFz(), 0.0);
            assertEquals(acceleration1, estimator.getStandardDeviationFzAsAcceleration());
            estimator.getStandardDeviationFzAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            final var aStdTriad1 = estimator.getStandardDeviationF();
            assertEquals(0.0, aStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, aStdTriad1.getValueZ(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, aStdTriad1.getUnit());
            final var aStdTriad2 = new AccelerationTriad();
            estimator.getStandardDeviationF(aStdTriad2);
            assertEquals(aStdTriad1, aStdTriad2);
            assertEquals(0.0, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
            assertEquals(acceleration1, estimator.getAverageAccelerometerStandardDeviationAsAcceleration());
            estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
            assertEquals(acceleration1, acceleration2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateXAsAngularSpeed());
            estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateYAsAngularSpeed());
            estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
            assertEquals(angularSpeed1, estimator.getStandardDeviationAngularRateZAsAngularSpeed());
            estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            final var wStdTriad1 = estimator.getStandardDeviationAngularRate();
            assertEquals(0.0, wStdTriad1.getValueX(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueY(), 0.0);
            assertEquals(0.0, wStdTriad1.getValueZ(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wStdTriad1.getUnit());
            final var wStdTriad2 = new AngularSpeedTriad();
            estimator.getStandardDeviationAngularRate(wStdTriad2);
            assertEquals(wStdTriad1, wStdTriad2);
            assertEquals(0.0, estimator.getAverageGyroscopeStandardDeviation(), 0.0);
            assertEquals(angularSpeed1, estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed());
            estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
            assertEquals(angularSpeed1, angularSpeed2);
            assertEquals(kinematics1, estimator.getStandardDeviationsAsBodyKinematics());
            estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);
            assertEquals(0.0, estimator.getPSDFx(), 0.0);
            assertEquals(0.0, estimator.getPSDFy(), 0.0);
            assertEquals(0.0, estimator.getPSDFz(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFx(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFy(), 0.0);
            assertEquals(0.0, estimator.getRootPSDFz(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateX(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateY(), 0.0);
            assertEquals(0.0, estimator.getRootPSDAngularRateZ(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getAccelerometerNoiseRootPSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoisePSD(), 0.0);
            assertEquals(0.0, estimator.getGyroNoiseRootPSD(), 0.0);
            assertEquals(m1, estimator.getAccelerometerBias());
            estimator.getAccelerometerBias(m2);
            assertEquals(m1, m2);
            assertEquals(m1, estimator.getGyroBias());
            estimator.getGyroBias(m2);
            assertEquals(m1, m2);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
            final var elapsedTime1 = estimator.getElapsedTime();
            assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
            final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
            estimator.getElapsedTime(elapsedTime2);
            assertEquals(elapsedTime1, elapsedTime2);
            assertFalse(estimator.isRunning());
            assertTrue(estimator.getExpectedKinematics().equals(expectedKinematics, ABSOLUTE_ERROR));
            estimator.getExpectedKinematics(kinematics2);
            assertTrue(expectedKinematics.equals(kinematics2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> new BodyKinematicsBiasEstimator(ecefPosition1, new CoordinateTransformation(
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME),
                            timeInterval, this));

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> new BodyKinematicsBiasEstimator(ecefPosition1, nedC,
                    -1.0, this));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        // set new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(1.0,
                estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTimeInterval(-1.0));
    }

    @Test
    void testGetTimeIntervalAsTime() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, time1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set new value
        final var time2 = new Time(1.0, TimeUnit.SECOND);
        estimator.setTimeInterval(time2);

        // check
        final var time3 = estimator.getTimeIntervalAsTime();
        final var time4 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getTimeIntervalAsTime(time4);

        assertEquals(time3, time4);
    }

    @Test
    void testGetSetEcefPosition() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var ecefPosition1 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(estimator.getEcefPosition(), ecefPosition1);

        // set new value
        final var ecefPosition2 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);
        estimator.setEcefPosition(ecefPosition2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
    }

    @Test
    void testSetEcefPosition1() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var ecefPosition1 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(estimator.getEcefPosition(), ecefPosition1);

        //  set new value
        estimator.setEcefPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        // check
        final var ecefPosition2 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
    }

    @Test
    void testSetEcefPosition2() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var ecefPosition1 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(estimator.getEcefPosition(), ecefPosition1);

        //  set new value
        final var distance = new Distance(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, DistanceUnit.METER);
        estimator.setEcefPosition(distance, distance, distance);

        // check
        final var ecefPosition2 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
    }

    @Test
    void testSetEcefPosition3() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var ecefPosition1 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(ecefPosition1, estimator.getEcefPosition());

        // set new value
        final var position = new InhomogeneousPoint3D(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);
        estimator.setEcefPosition(position);

        // check
        final var ecefPosition2 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
    }

    @Test
    void testGetEcefFrame() {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);
    }

    @Test
    void testGetNedFrame() {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var nedFrame1 = new NEDFrame();

        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetNedPosition() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var nedPosition1 = estimator.getNedPosition();

        assertEquals(0.0, nedPosition1.getLatitude(), ABSOLUTE_ERROR);
        assertEquals(0.0, nedPosition1.getLongitude(), ABSOLUTE_ERROR);
        assertEquals(0.0, nedPosition1.getHeight(), ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        estimator.setNedPosition(nedPosition2);

        // check
        final var nedPosition3 = estimator.getNedPosition();
        final var nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
    }

    @Test
    void testSetNedPosition1() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var nedPosition3 = estimator.getNedPosition();
        final var nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);
    }

    @Test
    void testSetNedPosition2() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition2 = new NEDPosition(latitude, longitude, heightDistance);

        final var nedPosition3 = estimator.getNedPosition();
        final var nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
    }

    @Test
    void testSetNedPosition3() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var nedPosition3 = estimator.getNedPosition();
        final var nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());
    }

    @Test
    void testGetSetEcefC() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefC1, estimator.getEcefC());

        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        estimator.setEcefC(ecefC2);

        // check
        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefC2, ecefC3);
        assertEquals(ecefC2, ecefC4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefC(new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testGetSetNedC() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC1 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);
        estimator.setNedC(nedC1);

        final var nedC2 = estimator.getNedC();
        final var nedC3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC3);

        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC3, ABSOLUTE_ERROR));

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedC(new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME)));
    }

    @Test
    void testSetNedPositionAndNedOrientation1() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition1 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC1 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        estimator.setNedPositionAndNedOrientation(nedPosition1, nedC1);

        // check
        final var nedPosition2 = estimator.getNedPosition();
        final var nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndNedOrientation(nedPosition1, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndNedOrientation2() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC1 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, nedC1);

        // check
        final var nedPosition1 = new NEDPosition(latitude, longitude, height);

        final var nedPosition2 = estimator.getNedPosition();
        final var nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndNedOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndNedOrientation3() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC1 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, nedC1);

        // check
        final var nedPosition1 = new NEDPosition(latitude, longitude, heightDistance);

        final var nedPosition2 = estimator.getNedPosition();
        final var nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndNedOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndNedOrientation4() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC1 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        estimator.setNedPositionAndNedOrientation(latitude, longitude, heightDistance, nedC1);

        // check
        final var nedPosition1 = new NEDPosition(latitude, longitude, heightDistance);

        final var nedPosition2 = estimator.getNedPosition();
        final var nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndNedOrientation(latitude, longitude, heightDistance,
                        new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation1() throws InvalidSourceAndDestinationFrameTypeException, 
            LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setEcefPositionAndEcefOrientation(ecefPosition2, ecefC2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndEcefOrientation(ecefPosition1, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation2() throws InvalidSourceAndDestinationFrameTypeException, 
            LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        final var x = ecefPosition2.getX();
        final var y = ecefPosition2.getY();
        final var z = ecefPosition2.getZ();
        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndEcefOrientation(x, y, z, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation3() throws InvalidSourceAndDestinationFrameTypeException, 
            LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        final var x = new Distance(ecefPosition2.getX(), DistanceUnit.METER);
        final var y = new Distance(ecefPosition2.getY(), DistanceUnit.METER);
        final var z = new Distance(ecefPosition2.getZ(), DistanceUnit.METER);
        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndEcefOrientation(x, y, z, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation4() throws InvalidSourceAndDestinationFrameTypeException, 
            LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        final var x = ecefPosition2.getX();
        final var y = ecefPosition2.getY();
        final var z = ecefPosition2.getZ();
        final var point = new InhomogeneousPoint3D(x, y, z);
        estimator.setEcefPositionAndEcefOrientation(point, ecefC2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(estimator.getExpectedKinematics(), expectedKinematics);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndEcefOrientation(point, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndEcefOrientation1() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(nedPosition2, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(estimator.getEcefC(), ecefC2);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndEcefOrientation(nedPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndEcefOrientation2() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndEcefOrientation3() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var heightDistance = new Distance(height, DistanceUnit.METER);
        final var nedPosition2 = new NEDPosition(latitude, longitude, heightDistance);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, 
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndEcefOrientation4() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES), 
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndNedOrientation1() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // set new values
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition2 = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            final var ecefPosition2 = ecefFrame2.getECEFPosition();
            final var ecefC2 = ecefFrame2.getCoordinateTransformation();

            estimator.setEcefPositionAndNedOrientation(ecefPosition2, nedC2);

            // check
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2, 5.0 * ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2, 5.0 * ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndNedOrientation(ecefPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndNedOrientation2() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var estimator = new BodyKinematicsBiasEstimator();

            // check default values
            final var nedFrame1 = new NEDFrame();
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            final var nedPosition1 = nedFrame1.getPosition();
            final var nedC1 = nedFrame1.getCoordinateTransformation();
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefC1 = ecefFrame1.getCoordinateTransformation();

            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
            assertEquals(ecefPosition1, estimator.getEcefPosition());
            assertEquals(ecefC1, estimator.getEcefC());

            // set new values
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition2 = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            final var ecefPosition2 = ecefFrame2.getECEFPosition();
            final var ecefC2 = ecefFrame2.getCoordinateTransformation();

            final var x = ecefPosition2.getX();
            final var y = ecefPosition2.getY();
            final var z = ecefPosition2.getZ();

            estimator.setEcefPositionAndNedOrientation(x, y, z, nedC2);

            // check
            if (!estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            if (!estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2,
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2,
                    LARGE_ABSOLUTE_ERROR));
            if (!estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, estimator.getEcefPosition());
            assertEquals(expectedKinematics, estimator.getExpectedKinematics());

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> estimator.setEcefPositionAndNedOrientation(x, y, z, new CoordinateTransformation(
                            FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSetEcefPositionAndNedOrientation3() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // set new values
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition2 = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            final var ecefPosition2 = ecefFrame2.getECEFPosition();
            final var ecefC2 = ecefFrame2.getCoordinateTransformation();

            final var x = new Distance(ecefPosition2.getX(), DistanceUnit.METER);
            final var y = new Distance(ecefPosition2.getY(), DistanceUnit.METER);
            final var z = new Distance(ecefPosition2.getZ(), DistanceUnit.METER);

            estimator.setEcefPositionAndNedOrientation(x, y, z, nedC2);

            // check
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, estimator.getEcefPosition());
            assertEquals(expectedKinematics, estimator.getExpectedKinematics());

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> estimator.setEcefPositionAndNedOrientation(x, y, z, new CoordinateTransformation(
                            FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSetEcefPositionAndNedOrientation4() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        final var x = ecefPosition2.getX();
        final var y = ecefPosition2.getY();
        final var z = ecefPosition2.getZ();
        final var point = new InhomogeneousPoint3D(x, y, z);

        estimator.setEcefPositionAndNedOrientation(point, nedC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertTrue(estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                estimator.getTimeInterval(), estimator.getEcefC(), estimator.getEcefC(), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, estimator.getEcefPosition());
        assertEquals(expectedKinematics, estimator.getExpectedKinematics());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndNedOrientation(point, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new BodyKinematicsBiasEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetAccelerometerBias() throws WrongSizeException {
        final var estimator = new BodyKinematicsBiasEstimator();

        final var accelerometerBias1 = new Matrix(3, 1);
        final var accelerometerBias2 = estimator.getAccelerometerBias();
        final var accelerometerBias3 = new Matrix(3, 1);
        estimator.getAccelerometerBias(accelerometerBias3);

        assertEquals(accelerometerBias1, accelerometerBias2);
        assertEquals(accelerometerBias1, accelerometerBias3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.getAccelerometerBias(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.getAccelerometerBias(m2));
    }

    @Test
    void testGetGyroBias() throws WrongSizeException {
        final var estimator = new BodyKinematicsBiasEstimator();

        final var gyroBias1 = new Matrix(3, 1);
        final var gyroBias2 = estimator.getGyroBias();
        final var gyroBias3 = new Matrix(3, 1);
        estimator.getGyroBias(gyroBias3);

        assertEquals(gyroBias1, gyroBias2);
        assertEquals(gyroBias1, gyroBias3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.getGyroBias(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.getGyroBias(m2));
    }

    @Test
    void testAddBodyKinematicsAndReset() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, 
            LockedException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new BodyKinematicsBiasEstimator(nedPosition, nedC, this);

        // Expected true kinematics for a static body at provided location and
        // orientation
        final var timeInterval = estimator.getTimeInterval();
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefC, ecefC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var lastKinematics = new BodyKinematics();
        for (var i = 0; i < N_SAMPLES; i++) {
            if (estimator.getLastBodyKinematics(lastKinematics)) {
                assertEquals(lastKinematics, estimator.getLastBodyKinematics());
                assertEquals(lastKinematics, kinematics);
            }

            final var random = new Random();
            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            estimator.addBodyKinematics(kinematics);

            assertTrue(estimator.getLastBodyKinematics(lastKinematics));
            assertEquals(lastKinematics, kinematics);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var biasFx = estimator.getBiasFx();
        final var biasFy = estimator.getBiasFy();
        final var biasFz = estimator.getBiasFz();

        final var biasAngularRateX = estimator.getBiasAngularRateX();
        final var biasAngularRateY = estimator.getBiasAngularRateY();
        final var biasAngularRateZ = estimator.getBiasAngularRateZ();

        assertEquals(biasFx, ba.getElementAtIndex(0), ACCELEROMETER_BIAS_ERROR);
        assertEquals(biasFy, ba.getElementAtIndex(1), ACCELEROMETER_BIAS_ERROR);
        assertEquals(biasFz, ba.getElementAtIndex(2), ACCELEROMETER_BIAS_ERROR);

        assertEquals(biasAngularRateX, bg.getElementAtIndex(0), GYRO_BIAS_ERROR);
        assertEquals(biasAngularRateY, bg.getElementAtIndex(1), GYRO_BIAS_ERROR);
        assertEquals(biasAngularRateZ, bg.getElementAtIndex(2), GYRO_BIAS_ERROR);

        final var biasFx1 = new Acceleration(biasFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var biasFx2 = estimator.getBiasFxAsAcceleration();
        final var biasFx3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getBiasFxAsAcceleration(biasFx3);

        assertEquals(biasFx1, biasFx2);
        assertEquals(biasFx1, biasFx3);

        final var biasFy1 = new Acceleration(biasFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var biasFy2 = estimator.getBiasFyAsAcceleration();
        final var biasFy3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getBiasFyAsAcceleration(biasFy3);

        assertEquals(biasFy1, biasFy2);
        assertEquals(biasFy1, biasFy3);

        final var biasFz1 = new Acceleration(biasFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var biasFz2 = estimator.getBiasFzAsAcceleration();
        final var biasFz3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getBiasFzAsAcceleration(biasFz3);

        assertEquals(biasFz1, biasFz2);
        assertEquals(biasFz1, biasFz3);

        final var biasAngularRateX1 = new AngularSpeed(biasAngularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var biasAngularRateX2 = estimator.getBiasAngularRateXAsAngularSpeed();
        final var biasAngularRateX3 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getBiasAngularRateXAsAngularSpeed(biasAngularRateX3);

        assertEquals(biasAngularRateX1, biasAngularRateX2);
        assertEquals(biasAngularRateX1, biasAngularRateX3);

        final var biasAngularRateY1 = new AngularSpeed(biasAngularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var biasAngularRateY2 = estimator.getBiasAngularRateYAsAngularSpeed();
        final var biasAngularRateY3 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getBiasAngularRateYAsAngularSpeed(biasAngularRateY3);

        assertEquals(biasAngularRateY1, biasAngularRateY2);
        assertEquals(biasAngularRateY1, biasAngularRateY3);

        final var biasAngularRateZ1 = new AngularSpeed(biasAngularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var biasAngularRateZ2 = estimator.getBiasAngularRateZAsAngularSpeed();
        final var biasAngularRateZ3 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getBiasAngularRateZAsAngularSpeed(biasAngularRateZ3);

        assertEquals(biasAngularRateZ1, biasAngularRateZ2);
        assertEquals(biasAngularRateZ1, biasAngularRateZ3);

        final var biasF1 = estimator.getBiasF();
        assertEquals(biasFx, biasF1.getValueX(), 0.0);
        assertEquals(biasFy, biasF1.getValueY(), 0.0);
        assertEquals(biasFz, biasF1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasF1.getUnit());
        final var biasF2 = new AccelerationTriad();
        estimator.getBiasF(biasF2);
        assertEquals(biasF1, biasF2);

        final var biasAngularRate1 = estimator.getBiasAngularRate();
        assertEquals(biasAngularRateX, biasAngularRate1.getValueX(), 0.0);
        assertEquals(biasAngularRateY, biasAngularRate1.getValueY(), 0.0);
        assertEquals(biasAngularRateZ, biasAngularRate1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasAngularRate1.getUnit());
        final var biasAngularRate2 = new AngularSpeedTriad();
        estimator.getBiasAngularRate(biasAngularRate2);
        assertEquals(biasAngularRate1, biasAngularRate2);

        final var biasKinematics1 = estimator.getBiasesAsBodyKinematics();
        final var biasKinematics2 = new BodyKinematics();
        estimator.getBiasesAsBodyKinematics(biasKinematics2);

        assertEquals(biasFx, biasKinematics1.getFx(), 0.0);
        assertEquals(biasFy, biasKinematics1.getFy(), 0.0);
        assertEquals(biasFz, biasKinematics1.getFz(), 0.0);
        assertEquals(biasAngularRateX, biasKinematics1.getAngularRateX(), 0.0);
        assertEquals(biasAngularRateY, biasKinematics1.getAngularRateY(), 0.0);
        assertEquals(biasAngularRateZ, biasKinematics1.getAngularRateZ(), 0.0);
        assertEquals(biasKinematics1, biasKinematics2);

        final var varianceFx = estimator.getVarianceFx();
        final var varianceFy = estimator.getVarianceFy();
        final var varianceFz = estimator.getVarianceFz();
        final var varianceAngularRateX = estimator.getVarianceAngularRateX();
        final var varianceAngularRateY = estimator.getVarianceAngularRateY();
        final var varianceAngularRateZ = estimator.getVarianceAngularRateZ();

        final var standardDeviationFx = estimator.getStandardDeviationFx();
        final var standardDeviationFy = estimator.getStandardDeviationFy();
        final var standardDeviationFz = estimator.getStandardDeviationFz();
        final var standardDeviationAngularRateX = estimator.getStandardDeviationAngularRateX();
        final var standardDeviationAngularRateY = estimator.getStandardDeviationAngularRateY();
        final var standardDeviationAngularRateZ = estimator.getStandardDeviationAngularRateZ();

        final var avgStdF = (standardDeviationFx + standardDeviationFy + standardDeviationFz) / 3.0;
        final var avgStdAngularRate = (standardDeviationAngularRateX
                + standardDeviationAngularRateY + standardDeviationAngularRateZ) / 3.0;

        assertEquals(avgStdF, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(avgStdAngularRate, estimator.getAverageGyroscopeStandardDeviation(), 0.0);

        assertEquals(Math.sqrt(varianceFx), standardDeviationFx, 0.0);
        assertEquals(Math.sqrt(varianceFy), standardDeviationFy, 0.0);
        assertEquals(Math.sqrt(varianceFz), standardDeviationFz, 0.0);
        assertEquals(Math.sqrt(varianceAngularRateX), standardDeviationAngularRateX, 0.0);
        assertEquals(Math.sqrt(varianceAngularRateY), standardDeviationAngularRateY, 0.0);
        assertEquals(Math.sqrt(varianceAngularRateZ), standardDeviationAngularRateZ, 0.0);

        final var standardDeviationFx1 = new Acceleration(standardDeviationFx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var standardDeviationFx2 = estimator.getStandardDeviationFxAsAcceleration();
        final var standardDeviationFx3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationFxAsAcceleration(standardDeviationFx3);

        assertEquals(standardDeviationFx1, standardDeviationFx2);
        assertEquals(standardDeviationFx1, standardDeviationFx3);

        final var standardDeviationFy1 = new Acceleration(standardDeviationFy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var standardDeviationFy2 = estimator.getStandardDeviationFyAsAcceleration();
        final var standardDeviationFy3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationFyAsAcceleration(standardDeviationFy3);

        assertEquals(standardDeviationFy1, standardDeviationFy2);
        assertEquals(standardDeviationFy1, standardDeviationFy3);

        final var standardDeviationFz1 = new Acceleration(standardDeviationFz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var standardDeviationFz2 = estimator.getStandardDeviationFzAsAcceleration();
        final var standardDeviationFz3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationFzAsAcceleration(standardDeviationFz3);

        assertEquals(standardDeviationFz1, standardDeviationFz2);
        assertEquals(standardDeviationFz1, standardDeviationFz3);

        final var standardDeviationF1 = estimator.getStandardDeviationF();
        assertEquals(standardDeviationF1.getValueX(), standardDeviationFx, 0.0);
        assertEquals(standardDeviationF1.getValueY(), standardDeviationFy, 0.0);
        assertEquals(standardDeviationF1.getValueZ(), standardDeviationFz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, standardDeviationF1.getUnit());
        final var standardDeviationF2 = new AccelerationTriad();
        estimator.getStandardDeviationF(standardDeviationF2);
        assertEquals(standardDeviationF1, standardDeviationF2);

        final var avgF1 = estimator.getAverageAccelerometerStandardDeviationAsAcceleration();
        final var avgF2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(avgF2);

        assertEquals(avgStdF, avgF1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgF1.getUnit());
        assertEquals(avgF1, avgF2);

        final var standardDeviationAngularRateX1 = new AngularSpeed(standardDeviationAngularRateX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final var standardDeviationAngularRateX2 = estimator.getStandardDeviationAngularRateXAsAngularSpeed();
        final var standardDeviationAngularRateX3 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(standardDeviationAngularRateX3);

        assertEquals(standardDeviationAngularRateX1, standardDeviationAngularRateX2);
        assertEquals(standardDeviationAngularRateX1, standardDeviationAngularRateX3);

        final var standardDeviationAngularRateY1 = new AngularSpeed(standardDeviationAngularRateY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final var standardDeviationAngularRateY2 = estimator.getStandardDeviationAngularRateYAsAngularSpeed();
        final var standardDeviationAngularRateY3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(standardDeviationAngularRateY3);

        assertEquals(standardDeviationAngularRateY1, standardDeviationAngularRateY2);
        assertEquals(standardDeviationAngularRateY1, standardDeviationAngularRateY3);

        final var standardDeviationAngularRateZ1 = new AngularSpeed(standardDeviationAngularRateZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final var standardDeviationAngularRateZ2 = estimator.getStandardDeviationAngularRateZAsAngularSpeed();
        final var standardDeviationAngularRateZ3 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(standardDeviationAngularRateZ3);

        assertEquals(standardDeviationAngularRateZ1, standardDeviationAngularRateZ2);
        assertEquals(standardDeviationAngularRateZ1, standardDeviationAngularRateZ3);

        final var standardDeviationAngularRate1 = estimator.getStandardDeviationAngularRate();
        assertEquals(standardDeviationAngularRate1.getValueX(), standardDeviationAngularRateX, 0.0);
        assertEquals(standardDeviationAngularRate1.getValueY(), standardDeviationAngularRateY, 0.0);
        assertEquals(standardDeviationAngularRate1.getValueZ(), standardDeviationAngularRateZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationAngularRate1.getUnit());
        final var standardDeviationAngularRate2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularRate(standardDeviationAngularRate2);
        assertEquals(standardDeviationAngularRate1, standardDeviationAngularRate2);

        final var avgAngularRate1 = estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed();
        final var avgAngularRate2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(avgAngularRate2);

        assertEquals(avgAngularRate1.getValue().doubleValue(), avgStdAngularRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgAngularRate1.getUnit());
        assertEquals(avgAngularRate1, avgAngularRate2);

        final var standardDeviationKinematics1 = estimator.getStandardDeviationsAsBodyKinematics();
        final var standardDeviationKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationsAsBodyKinematics(standardDeviationKinematics2);

        assertEquals(standardDeviationFx, standardDeviationKinematics1.getFx(), 0.0);
        assertEquals(standardDeviationFy, standardDeviationKinematics1.getFy(), 0.0);
        assertEquals(standardDeviationFz, standardDeviationKinematics1.getFz(), 0.0);
        assertEquals(standardDeviationAngularRateX, standardDeviationKinematics1.getAngularRateX(), 0.0);
        assertEquals(standardDeviationAngularRateY, standardDeviationKinematics1.getAngularRateY(), 0.0);
        assertEquals(standardDeviationAngularRateZ, standardDeviationKinematics1.getAngularRateZ(), 0.0);
        assertEquals(standardDeviationKinematics1, standardDeviationKinematics2);

        final var psdFx = estimator.getPSDFx();
        final var psdFy = estimator.getPSDFy();
        final var psdFz = estimator.getPSDFz();
        final var psdAngularRateX = estimator.getPSDAngularRateX();
        final var psdAngularRateY = estimator.getPSDAngularRateY();
        final var psdAngularRateZ = estimator.getPSDAngularRateZ();

        assertEquals(varianceFx * timeInterval, psdFx, 0.0);
        assertEquals(varianceFy * timeInterval, psdFy, 0.0);
        assertEquals(varianceFz * timeInterval, psdFz, 0.0);
        assertEquals(varianceAngularRateX * timeInterval, psdAngularRateX, 0.0);
        assertEquals(varianceAngularRateY * timeInterval, psdAngularRateY, 0.0);
        assertEquals(varianceAngularRateZ * timeInterval, psdAngularRateZ, 0.0);

        final var rootPsdFx = estimator.getRootPSDFx();
        final var rootPsdFy = estimator.getRootPSDFy();
        final var rootPsdFz = estimator.getRootPSDFz();
        final var rootPsdAngularRateX = estimator.getRootPSDAngularRateX();
        final var rootPsdAngularRateY = estimator.getRootPSDAngularRateY();
        final var rootPsdAngularRateZ = estimator.getRootPSDAngularRateZ();

        assertEquals(rootPsdFx, accelNoiseRootPSD, ACCELEROMETER_NOISE_ROOT_PSD_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, ACCELEROMETER_NOISE_ROOT_PSD_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, ACCELEROMETER_NOISE_ROOT_PSD_ERROR);

        assertEquals(rootPsdAngularRateX, gyroNoiseRootPSD, GYRO_NOISE_ROOT_PSD_ERROR);
        assertEquals(rootPsdAngularRateY, gyroNoiseRootPSD, GYRO_NOISE_ROOT_PSD_ERROR);
        assertEquals(rootPsdAngularRateZ, gyroNoiseRootPSD, GYRO_NOISE_ROOT_PSD_ERROR);

        assertEquals(Math.sqrt(psdFx), rootPsdFx, 0.0);
        assertEquals(Math.sqrt(psdFy), rootPsdFy, 0.0);
        assertEquals(Math.sqrt(psdFz), rootPsdFz, 0.0);
        assertEquals(Math.sqrt(psdAngularRateX), rootPsdAngularRateX, 0.0);
        assertEquals(Math.sqrt(psdAngularRateY), rootPsdAngularRateY, 0.0);
        assertEquals(Math.sqrt(psdAngularRateZ), rootPsdAngularRateZ, 0.0);

        final var accelerometerNoisePsd = (psdFx + psdFy + psdFz) / 3.0;
        final var gyroNoisePsd = (psdAngularRateX + psdAngularRateY + psdAngularRateZ) / 3.0;

        assertEquals(accelerometerNoisePsd, estimator.getAccelerometerNoisePSD(), 0.0);
        assertEquals(gyroNoisePsd, estimator.getGyroNoisePSD(), 0.0);

        final var estimatedAccelNoiseRootPSD = estimator.getAccelerometerNoiseRootPSD();
        final var estimatedGyroNoiseRootPSD = estimator.getGyroNoiseRootPSD();
        assertEquals(Math.sqrt(psdFx + psdFy + psdFz), estimatedAccelNoiseRootPSD, 0.0);
        assertEquals(Math.sqrt(rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy + rootPsdFz * rootPsdFz),
                estimatedAccelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(Math.sqrt(psdAngularRateX + psdAngularRateY + psdAngularRateZ),
                estimatedGyroNoiseRootPSD, 0.0);
        assertEquals(Math.sqrt(rootPsdAngularRateX * rootPsdAngularRateX + rootPsdAngularRateY * rootPsdAngularRateY
                        + rootPsdAngularRateZ * rootPsdAngularRateZ), estimatedGyroNoiseRootPSD, ABSOLUTE_ERROR);

        assertEquals(accelNoiseRootPSD, Math.sqrt(accelerometerNoisePsd), ACCELEROMETER_NOISE_ROOT_PSD_ERROR);
        assertEquals(gyroNoiseRootPSD, Math.sqrt(gyroNoisePsd), GYRO_NOISE_ROOT_PSD_ERROR);

        final var accelerometerBias1 = Matrix.newFromArray(new double[]{biasFx, biasFy, biasFz});
        final var accelerometerBias2 = estimator.getAccelerometerBias();
        final var accelerometerBias3 = new Matrix(3, 1);
        estimator.getAccelerometerBias(accelerometerBias3);

        assertEquals(accelerometerBias1, accelerometerBias2);
        assertEquals(accelerometerBias1, accelerometerBias3);

        final var gyroBias1 = Matrix.newFromArray(new double[]{biasAngularRateX, biasAngularRateY, biasAngularRateZ});
        final var gyroBias2 = estimator.getGyroBias();
        final var gyroBias3 = new Matrix(3, 1);
        estimator.getGyroBias(gyroBias3);

        assertEquals(gyroBias1, gyroBias2);
        assertEquals(gyroBias1, gyroBias3);

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());

        final var expectedKinematics1 = estimator.getExpectedKinematics();
        final var expectedKinematics2 = new BodyKinematics();
        estimator.getExpectedKinematics(expectedKinematics2);

        assertEquals(trueKinematics, expectedKinematics1);
        assertEquals(trueKinematics, expectedKinematics2);

        // reset
        assertTrue(estimator.reset());

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());
        assertEquals(1, reset);
    }

    @Override
    public void onStart(final BodyKinematicsBiasEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onBodyKinematicsAdded(final BodyKinematicsBiasEstimator estimator) {
        if (bodyKinematicsAdded == 0) {
            checkLocked(estimator);
        }
        bodyKinematicsAdded++;
    }

    @Override
    public void onReset(final BodyKinematicsBiasEstimator estimator) {
        checkLocked(estimator);
        reset++;
    }

    private void reset() {
        start = 0;
        bodyKinematicsAdded = 0;
        reset = 0;
    }

    private static void checkLocked(final BodyKinematicsBiasEstimator estimator) {
        final var ecefPosition = new ECEFPosition();
        final var distance = new Distance(0.0, DistanceUnit.METER);
        final var point = Point3D.create();
        final var nedPosition = new NEDPosition();
        final var angle = new Angle(0.0, AngleUnit.RADIANS);

        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition(ecefPosition));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition(distance, distance, distance));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition(point));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(nedPosition));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(angle, angle, 0.0));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(angle, angle, distance));
        assertThrows(LockedException.class, () -> estimator.setEcefC(null));
        assertThrows(LockedException.class, () -> estimator.setNedC(null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(nedPosition, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(angle, angle, 0.0,
                null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(angle, angle, distance,
                null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(ecefPosition,
                null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(0.0, 0.0, 0.0,
                null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(
                distance, distance, distance, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(point, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(nedPosition, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(angle, angle, 0.0,
                null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(angle, angle, distance,
                null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(ecefPosition, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(0.0, 0.0, 0.0,
                null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(
                distance, distance, distance, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(point, null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.addBodyKinematics(null));
        assertThrows(LockedException.class, () -> assertFalse(estimator.reset()));
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private static Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private static Matrix generateMa() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        final var tmp = DEG_TO_RAD / (3600 * 9.80665);
        result.fromArray(new double[]{
                0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                0.3 * tmp, 1.1 * tmp, -1.3 * tmp
        }, false);

        return result;
    }

    private static double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private static double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }
}
