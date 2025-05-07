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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.AxisRotation3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class KnownBiasTurntableGyroscopeCalibratorTest implements KnownBiasTurntableGyroscopeCalibratorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

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

    private static final double MIN_ROTATION_RATE_DEGREES_PER_SECOND = 90.0;
    private static final double MAX_ROTATION_RATE_DEGREES_PER_SECOND = 180.0;

    private static final double MIN_TIME_INTERVAL = 0.01;
    private static final double MAX_TIME_INTERVAL = 0.1;

    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;

    private static final int TIMES = 100;

    private int calibrateStart;
    private int calibrateEnd;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), bg1);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(Constants.EARTH_ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(Constants.EARTH_ROTATION_RATE, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(TurntableGyroscopeCalibrator.DEFAULT_TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        final var time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(TurntableGyroscopeCalibrator.DEFAULT_TIME_INTERVAL, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(calibrator.getMeasurements());
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, bg, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(timeInterval, calibrator.getTimeInterval(), 0.0);
        final var time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval, 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, m6));
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, bg, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, bg, mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, m6, this));
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, bias, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, m4));
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, bias, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, m4, this));
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, bias, mg, gg, accelerometerBias, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, new double[1], mg, gg, accelerometerBias, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, m1, gg, accelerometerBias, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, m2, gg, accelerometerBias, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, m3, accelerometerBias, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, m4, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, accelerometerBias, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, accelerometerBias, m6));
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, bias, mg, gg, accelerometerBias, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, new double[1], mg, gg, accelerometerBias, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, m1, gg, accelerometerBias, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, m2, gg, accelerometerBias, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, m3, accelerometerBias, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, m4, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, accelerometerBias, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, accelerometerBias, m6, this));
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bg, mg, gg, ba, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, ba, m10));
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bg, mg, gg, ba, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(
                ecefPosition, rotationRate, timeInterval, measurements, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, ba, m10, this));
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, false, false, bg, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false, bg,
                mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, m1,
                mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, m2,
                mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m6));
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, false, false, bg, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false, bg,
                mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, m1,
                mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, m2,
                mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m6, this));
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, false, false, bias, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false,
                bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(
                ecefPosition, rotationRate, timeInterval, measurements, true,
                false, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m4));
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, false, false, bias, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false,
                bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false,
                new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m4, this));
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, false, false, bias, mg, gg, accelerometerBias,
                ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false,
                bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false,
                new double[1], mg, gg, accelerometerBias, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false,
                bias, m1, gg, accelerometerBias, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m2, gg, accelerometerBias, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m3, accelerometerBias, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m4, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, accelerometerBias, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, accelerometerBias, m6));
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, false, false, bias, mg, gg, accelerometerBias,
                ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false,
                bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false,
                new double[1], mg, gg, accelerometerBias, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m1, gg, accelerometerBias, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m2, gg, accelerometerBias, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m3, accelerometerBias, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m4, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, accelerometerBias, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, accelerometerBias, m6, this));
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, false, false, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, ba, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false, bg,
                mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false,
                m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(
                ecefPosition, rotationRate, timeInterval, measurements, true,
                false, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(
                ecefPosition, rotationRate, timeInterval, measurements, true,
                false, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, ba, m10));
    }

    @Test
    void testConstructor17() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate, timeInterval,
                measurements, false, false, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, ba, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false,
                bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false,
                m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false,
                m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg, mg,
                m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg, mg,
                m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg, mg,
                gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg, mg,
                gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg, mg,
                gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg, mg,
                gg, ba, m10, this));
    }

    @Test
    void testConstructor18() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, bg, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, m6));
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, bg, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, bg, mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, m6, this));
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, bias, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, m4));
    }

    @Test
    void testConstructor21() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, bias, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, m4, this));
    }

    @Test
    void testConstructor22() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, bias, mg, gg, accelerometerBias, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, new double[1], mg, gg, accelerometerBias, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, m1, gg, accelerometerBias, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, m2, gg, accelerometerBias, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, m3, accelerometerBias, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, m4, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, accelerometerBias, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, accelerometerBias, m6));
    }

    @Test
    void testConstructor23() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, bias, mg, gg, accelerometerBias, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, new double[1], mg, gg, accelerometerBias, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, m1, gg, accelerometerBias, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, m2, gg, accelerometerBias, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, m3, accelerometerBias, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, m4, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, accelerometerBias, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bias, mg, gg, accelerometerBias, m6, this));
    }

    @Test
    void testConstructor24() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bg, mg, gg, ba, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, ba, m10));
    }

    @Test
    void testConstructor25() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bg, mg, gg, ba, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, ba, m10, this));
    }

    @Test
    void testConstructor26() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, false, false, bg, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, true, false, bg,
                mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, m1,
                mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, m2,
                mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m6));
    }

    @Test
    void testConstructor27() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, false, false, bg, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, true, false, bg,
                mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, m1,
                mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, m2,
                mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m6, this));
    }

    @Test
    void testConstructor28() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, false, false, bias, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, true, false,
                bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m4));
    }

    @Test
    void testConstructor29() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, false, false, bias, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(new Matrix(3, 3), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, true, false, bias,
                mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m4, this));
    }

    @Test
    void testConstructor30() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, false, false, bias, mg, gg, accelerometerBias,
                ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, true, false, bias,
                mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                new double[1], mg, gg, accelerometerBias, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m1, gg, accelerometerBias, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m2, gg, accelerometerBias, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m3, accelerometerBias, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m4, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, accelerometerBias, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, accelerometerBias, m6));
    }

    @Test
    void testConstructor31() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, false, false, bias, mg, gg, accelerometerBias,
                ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, 0.0, measurements, true, false, bias,
                mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                new double[1], mg, gg, accelerometerBias, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m1, gg, accelerometerBias, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m2, gg, accelerometerBias, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m3, accelerometerBias, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, m4, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, accelerometerBias, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                mg, gg, accelerometerBias, m6, this));
    }

    @Test
    void testConstructor32() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, false, false, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, ba, m10));
    }

    @Test
    void testConstructor33() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var rotationRate = getTurntableRotationRate();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var bg = generateBg();
        final var mg = generateCommonAxisMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var bias = bg.getBuffer();

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var ba = generateBa();
        final var ma = generateMa();
        final var accelerometerBias = ba.getBuffer();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                measurements, false, false, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final var accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
        final var angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final var angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final var angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(), rotationRate, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final var rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false,
                bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownBiasTurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg,
                mg, gg, ba, m10, this));
    }

    @Test
    void testGetSetAccelerometerBiasX() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);

        // set new value
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);

        calibrator.setAccelerometerBiasX(bax);

        // check
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
    }

    @Test
    void testGetSetAccelerometerBiasY() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);

        // set new value
        final var ba = generateBa();
        final var bay = ba.getElementAtIndex(1);

        calibrator.setAccelerometerBiasY(bay);

        // check
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
    }

    @Test
    void testGetSetAccelerometerBiasZ() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);

        // set new value
        final var ba = generateBa();
        final var baz = ba.getElementAtIndex(2);

        calibrator.setAccelerometerBiasZ(baz);

        // check
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
    }

    @Test
    void testGetSetAccelerometerBiasXAsAcceleration() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final var acceleration1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set new value
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var acceleration2 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasX(acceleration2);

        // check
        final var acceleration3 = calibrator.getAccelerometerBiasXAsAcceleration();
        final var acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    void testGetSetAccelerometerBiasYAsAcceleration() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final var acceleration1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set new value
        final var ba = generateBa();
        final var bay = ba.getElementAtIndex(1);
        final var acceleration2 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasY(acceleration2);

        // check
        final var acceleration3 = calibrator.getAccelerometerBiasYAsAcceleration();
        final var acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    void testGetSetAccelerometerBiasZAsAcceleration() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final var acceleration1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set new value
        final var ba = generateBa();
        final var baz = ba.getElementAtIndex(2);
        final var acceleration2 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasZ(acceleration2);

        // check
        final var acceleration3 = calibrator.getAccelerometerBiasZAsAcceleration();
        final var acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    void testSetAccelerometerBias1() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        
        // set new values
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        calibrator.setAccelerometerBias(bax, bay, baz);

        // check
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
    }

    @Test
    void testSetAccelerometerBias2() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);

        // set new values
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var accelerationX = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setAccelerometerBias(accelerationX, accelerationY, accelerationZ);

        // check
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
    }

    @Test
    void testGetSetAccelerometerBiasArray() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertArrayEquals(new double[3], calibrator.getAccelerometerBias(), 0.0);

        // set new value
        final var ba = generateBa();
        final var bias = ba.getBuffer();

        calibrator.setAccelerometerBias(bias);

        // check
        final var bias1 = calibrator.getAccelerometerBias();
        final var bias2 = new double[3];
        calibrator.getAccelerometerBias(bias2);

        assertArrayEquals(bias, bias1, 0.0);
        assertArrayEquals(bias, bias2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccelerometerBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setAccelerometerBias(new double[1]));
    }

    @Test
    void testGetSetAccelerometerBiasAsMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getAccelerometerBiasAsMatrix());

        // set new value
        final var ba = generateBa();
        calibrator.setAccelerometerBias(ba);

        // check
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);

        assertEquals(ba, ba1);
        assertEquals(ba, ba2);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccelerometerBiasAsMatrix(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccelerometerBiasAsMatrix(m2));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setAccelerometerBias(m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setAccelerometerBias(m4));
    }

    @Test
    void testGetSetAccelerometerSx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);

        // set new value
        final var ma = generateMa();
        final var asx = ma.getElementAt(0, 0);

        calibrator.setAccelerometerSx(asx);

        // check
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
    }

    @Test
    void testGetSetAccelerometerSy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);

        // set new value
        final var ma = generateMa();
        final var asy = ma.getElementAt(1, 1);

        calibrator.setAccelerometerSy(asy);

        // check
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
    }

    @Test
    void testGetSetAccelerometerSz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);

        // set new value
        final var ma = generateMa();
        final var asz = ma.getElementAt(2, 2);

        calibrator.setAccelerometerSz(asz);

        // check
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMxy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);

        // set new value
        final var ma = generateMa();
        final var amxy = ma.getElementAt(0, 1);

        calibrator.setAccelerometerMxy(amxy);

        // check
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMxz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);

        // set new value
        final var ma = generateMa();
        final var amxz = ma.getElementAt(0, 2);

        calibrator.setAccelerometerMxz(amxz);

        // check
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMyx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);

        // set new value
        final var ma = generateMa();
        final var amyx = ma.getElementAt(1, 0);

        calibrator.setAccelerometerMyx(amyx);

        // check
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMyz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);

        // set new value
        final var ma = generateMa();
        final var amyz = ma.getElementAt(1, 2);

        calibrator.setAccelerometerMyz(amyz);

        // check
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMzx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);

        // set new value
        final var ma = generateMa();
        final var amzx = ma.getElementAt(2, 0);

        calibrator.setAccelerometerMzx(amzx);

        // check
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMzy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new value
        final var ma = generateMa();
        final var amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMzy(amzy);

        // check
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
    }

    @Test
    void testGetSetAccelerometerScalingFactors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);

        // set new values
        final var ma = generateMa();
        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);

        calibrator.setAccelerometerScalingFactors(asx, asy, asz);

        // check
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
    }

    @Test
    void testGetSetAccelerometerCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new values
        final var ma = generateMa();
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerCrossCouplingErrors(amxy, amxz, amyx, amyz, amzx, amzy);

        // check
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
    }

    @Test
    void testGetSetAccelerometerScalingFactorsAndCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new values
        final var ma = generateMa();
        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(asx, asy, asz, amxy, amxz, amyx, amyz, amzx,
                amzy);

        // check
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMa() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new values
        final var ma = generateMa();
        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMa(ma);

        // check
        final var ma1 = calibrator.getAccelerometerMa();
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);

        assertEquals(ma, ma1);
        assertEquals(ma, ma2);

        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccelerometerMa(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccelerometerMa(m2));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setAccelerometerMa(m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setAccelerometerMa(m4));
    }

    @Test
    void testGetSetBiasX() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasX(), 0.0);

        // set new value
        final var bg = generateBg();
        final var bx = bg.getElementAtIndex(0);

        calibrator.setBiasX(bx);

        // check
        assertEquals(bx, calibrator.getBiasX(), 0.0);
    }

    @Test
    void testGetSetBiasY() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasY(), 0.0);

        // set new value
        final var bg = generateBg();
        final var by = bg.getElementAtIndex(1);

        calibrator.setBiasY(by);

        // check
        assertEquals(by, calibrator.getBiasY(), 0.0);
    }

    @Test
    void testGetSetBiasZ() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new value
        final var bg = generateBg();
        final var bz = bg.getElementAtIndex(2);

        calibrator.setBiasZ(bz);

        // check
        assertEquals(bz, calibrator.getBiasZ(), 0.0);
    }

    @Test
    void testGetSetBiasAngularSpeedX() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final var angularSpeed1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());

        // set new value
        final var bg = generateBg();
        final var bx = bg.getElementAtIndex(0);
        final var angularSpeed2 = new AngularSpeed(bx, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasX(angularSpeed2);

        // check
        final var angularSpeed3 = calibrator.getBiasAngularSpeedX();
        final var angularSpeed4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    void testGetSetBiasAngularSpeedY() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final var angularSpeed1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());

        // set new value
        final var bg = generateBg();
        final var by = bg.getElementAtIndex(1);
        final var angularSpeed2 = new AngularSpeed(by, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasY(angularSpeed2);

        // check
        final var angularSpeed3 = calibrator.getBiasAngularSpeedY();
        final var angularSpeed4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    void testGetSetBiasAngularSpeedZ() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final var angularSpeed1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());

        // set new value
        final var bg = generateBg();
        final var bz = bg.getElementAtIndex(2);
        final var angularSpeed2 = new AngularSpeed(bz, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasZ(angularSpeed2);

        // check
        final var angularSpeed3 = calibrator.getBiasAngularSpeedZ();
        final var angularSpeed4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    void testSetBiasCoordinates1() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new values
        final var bg = generateBg();
        final var bx = bg.getElementAtIndex(0);
        final var by = bg.getElementAtIndex(1);
        final var bz = bg.getElementAtIndex(2);

        calibrator.setBiasCoordinates(bx, by, bz);

        // check
        assertEquals(bx, calibrator.getBiasX(), 0.0);
        assertEquals(by, calibrator.getBiasY(), 0.0);
        assertEquals(bz, calibrator.getBiasZ(), 0.0);
    }

    @Test
    void testSetBiasCoordinates2() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new values
        final var bg = generateBg();
        final var bx = bg.getElementAtIndex(0);
        final var by = bg.getElementAtIndex(1);
        final var bz = bg.getElementAtIndex(2);

        final var angularSpeedX = new AngularSpeed(bx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeedY = new AngularSpeed(by, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeedZ = new AngularSpeed(bz, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasCoordinates(angularSpeedX, angularSpeedY, angularSpeedZ);

        // check
        assertEquals(bx, calibrator.getBiasX(), 0.0);
        assertEquals(by, calibrator.getBiasY(), 0.0);
        assertEquals(bz, calibrator.getBiasZ(), 0.0);
    }

    @Test
    void testGetSetBiasAsTriad() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        final var triad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new value
        final var bg = generateBg();
        final var triad2 = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND);
        triad2.setValueCoordinates(bg);

        calibrator.setBias(triad2);

        // check
        final var triad3 = calibrator.getBiasAsTriad();
        final var triad4 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var sx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(sx);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var sy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(sy);

        // check
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var sz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(sz);

        // check
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var mxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var mxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var myx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(myx);

        // check
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var myz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(myz);

        // check
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var mzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final var mg = generateCommonAxisMg();
        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testSetInitialCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final var mg = generateCommonAxisMg();
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        calibrator.setInitialCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testSetInitialScalingFactorsAndCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final var mg = generateCommonAxisMg();
        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testGetSetBiasArray() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertArrayEquals(new double[3], calibrator.getBias(), 0.0);

        // set new value
        final var bg = generateBg();
        final var bias = bg.getBuffer();

        calibrator.setBias(bias);

        // check
        final var bias1 = calibrator.getBias();
        final var bias2 = new double[3];
        calibrator.getBias(bias2);

        assertArrayEquals(bias, bias1, 0.0);
        assertArrayEquals(bias, bias2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setBias(new double[1]));
    }

    @Test
    void testGetSetBiasAsMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getBiasAsMatrix());

        // set new value
        final var bg = generateBg();
        final var bx = bg.getElementAtIndex(0);
        final var by = bg.getElementAtIndex(1);
        final var bz = bg.getElementAtIndex(2);

        calibrator.setBias(bg);

        // check
        final var bg1 = calibrator.getBiasAsMatrix();
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);

        assertEquals(bg, bg1);
        assertEquals(bg, bg2);

        assertEquals(bx, calibrator.getBiasX(), 0.0);
        assertEquals(by, calibrator.getBiasY(), 0.0);
        assertEquals(bz, calibrator.getBiasZ(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getBiasAsMatrix(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getBiasAsMatrix(m2));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setBias(m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setBias(m4));
    }

    @Test
    void testGetSetInitialMg() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(new Matrix(3, 3), calibrator.getInitialMg());

        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new value
        final var mg = generateCommonAxisMg();
        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMg(mg);

        // check
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);

        assertEquals(mg, mg1);
        assertEquals(mg, mg2);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialMg(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialMg(m2));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialMg(m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialMg(m4));
    }

    @Test
    void testGetSetInitialGg() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(new Matrix(3, 3), calibrator.getInitialGg());

        // set new value
        final var gg = generateGg();
        calibrator.setInitialGg(gg);

        // check
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);

        assertEquals(gg, gg1);
        assertEquals(gg, gg2);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialGg(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialGg(m2));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialGg(m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialGg(m4));
    }

    @Test
    void testGetSetTurntableRotationRate() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(Constants.EARTH_ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);

        // set new value
        final var rotationRate = getTurntableRotationRate();
        calibrator.setTurntableRotationRate(rotationRate);

        // check
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setTurntableRotationRate(0.0));
    }

    @Test
    void testGetSetTurntableRotationRateAsAngularSpeed() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        final var rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(Constants.EARTH_ROTATION_RATE, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());

        // set new value
        final var rotationRate = getTurntableRotationRate();
        final var rotationRate2 = new AngularSpeed(rotationRate, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setTurntableRotationRate(rotationRate2);

        // check
        final var rotation3 = calibrator.getTurntableRotationRateAsAngularSpeed();
        final var rotation4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotation4);

        assertEquals(rotation3, rotation4);

        // Force IllegalArgumentException
        final var w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setTurntableRotationRate(w));
    }

    @Test
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testGetSetEcefPosition() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getEcefPosition());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        calibrator.setPosition(ecefPosition);

        // check
        assertSame(ecefPosition, calibrator.getEcefPosition());
    }

    @Test
    void testGetSetNedPosition() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        calibrator.setPosition(nedPosition);

        // check
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final var position2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(position2));
        assertTrue(nedPosition.equals(position2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertFalse(calibrator.isCommonAxisUsed());
    }

    @Test
    void testIsSetGDependentCrossBiasesEstimated() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetMinimumRequiredMeasurementsOrSequences() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertEquals(7, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertTrue(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(true);

        // check
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    void testIsReady() throws LockedException {
        final var calibrator = new KnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isReady());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());

        // set empty measurements
        final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
        calibrator.setMeasurements(measurements);

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements
        for (var i = 0; i < KnownBiasTurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS_AND_CROSS_BIASES; 
             i++) {
            measurements.add(new StandardDeviationBodyKinematics());
        }

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, 
            InvalidRotationMatrixException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var rotationRate = 100.0 * getTurntableRotationRate();
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateCommonAxisMg();
            final var gg = new Matrix(3, 3);
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final var accelNoiseRootPSD = 0.0;
            final var gyroNoiseRootPSD = 0.0;
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);
            
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS; i++) {
                final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME, 
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final var rot1 = nedC1.asRotation();
                final var axis1 = rot1.getRotationAxis();
                var angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final var x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final var rot = new AxisRotation3D(axis1, angleIncrement);
                final var rot2 = rot1.combineAndReturnNew(rot);
                final var nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, 
                        ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(new ECEFPosition(), rotationRate,
                    timeInterval, measurements, true, false, bg, mg, gg, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralAndGDependentCrossBiasesDisabledAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var rotationRate = 100.0 * getTurntableRotationRate();
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateGeneralMg();
            final var gg = new Matrix(3, 3);
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final var accelNoiseRootPSD = 0.0;
            final var gyroNoiseRootPSD = 0.0;
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            var timeInterval = TIME_INTERVAL_SECONDS;
            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_GENERAL; i++) {
                final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final var rot1 = nedC1.asRotation();
                final var axis1 = rot1.getRotationAxis();
                var angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final var x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final var rot = new AxisRotation3D(axis1, angleIncrement);
                final var rot2 = rot1.combineAndReturnNew(rot);
                final var nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                        ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(new ECEFPosition(), rotationRate,
                    timeInterval, measurements, false, false, bg, mg, gg,
                    this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var rotationRate = 100.0 * getTurntableRotationRate();
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateCommonAxisMg();
            final var gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final var accelNoiseRootPSD = 0.0;
            final var gyroNoiseRootPSD = 0.0;
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            var timeInterval = TIME_INTERVAL_SECONDS;
            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS_AND_CROSS_BIASES; i++) {
                final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final var rot1 = nedC1.asRotation();
                final var axis1 = rot1.getRotationAxis();
                var angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final var x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final var rot = new AxisRotation3D(axis1, angleIncrement);
                final var rot2 = rot1.combineAndReturnNew(rot);
                final var nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                        ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(new ECEFPosition(), rotationRate,
                    timeInterval, measurements, true, true, bg, mg, gg, ba,
                    ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralAndGDependentCrossBiasesEnabledAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var rotationRate = 100.0 * getTurntableRotationRate();
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateGeneralMg();
            final var gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final var accelNoiseRootPSD = 0.0;
            final var gyroNoiseRootPSD = 0.0;
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            var timeInterval = TIME_INTERVAL_SECONDS;
            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES; i++) {
                final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final var rot1 = nedC1.asRotation();
                final var axis1 = rot1.getRotationAxis();
                var angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final var x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final var rot = new AxisRotation3D(axis1, angleIncrement);
                final var rot2 = rot1.combineAndReturnNew(rot);
                final var nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                        ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors,
                        random);

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(new ECEFPosition(), rotationRate,
                    timeInterval, measurements, false, true, bg, mg, gg, ba,
                    ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledLargeNumberOfMeasurementsWithNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var rotationRate = 100.0 * getTurntableRotationRate();
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateCommonAxisMg();
            final var gg = new Matrix(3, 3);
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

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            var timeInterval = TIME_INTERVAL_SECONDS;
            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {
                final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final var rot1 = nedC1.asRotation();
                final var axis1 = rot1.getRotationAxis();
                var angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final var x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final var rot = new AxisRotation3D(axis1, angleIncrement);
                final var rot2 = rot1.combineAndReturnNew(rot);
                final var nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                        ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final var initialBg = new Matrix(3, 1);
            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                    measurements, true, false, initialBg, initialMg,
                    initialGg, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralAndGDependentCrossBiasesDisabledLargeNumberOfMeasurementsWithNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var rotationRate = 100.0 * getTurntableRotationRate();
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateGeneralMg();
            final var gg = new Matrix(3, 3);
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

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            var timeInterval = TIME_INTERVAL_SECONDS;
            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {
                final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final var rot1 = nedC1.asRotation();
                final var axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final var x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final var rot = new AxisRotation3D(axis1, angleIncrement);
                final var rot2 = rot1.combineAndReturnNew(rot);
                final var nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                        ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final var initialBg = new Matrix(3, 1);
            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                    measurements, false, false, initialBg, initialMg,
                    initialGg, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledLargeNumberOfMeasurementsWithNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var rotationRate = 100.0 * getTurntableRotationRate();
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateCommonAxisMg();
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

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            var timeInterval = TIME_INTERVAL_SECONDS;
            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {
                final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final var rot1 = nedC1.asRotation();
                final var axis1 = rot1.getRotationAxis();
                var angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final var x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final var rot = new AxisRotation3D(axis1, angleIncrement);
                final var rot2 = rot1.combineAndReturnNew(rot);
                final var nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                        ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors,
                        random);

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final var initialBg = new Matrix(3, 1);
            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                    measurements, true, true, initialBg, initialMg, initialGg,
                    ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralAndGDependentCrossBiasesEnabledLargeNumberOfMeasurementsWithNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var rotationRate = 100.0 * getTurntableRotationRate();
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateGeneralMg();
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

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            var timeInterval = TIME_INTERVAL_SECONDS;
            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {
                final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final var rot1 = nedC1.asRotation();
                final var axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final var x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final var rot = new AxisRotation3D(axis1, angleIncrement);
                final var rot2 = rot1.combineAndReturnNew(rot);
                final var nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval,
                        ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors,
                        random);

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final var initialBg = new Matrix(3, 1);
            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var calibrator = new KnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate, timeInterval,
                    measurements, false, true, initialBg, initialMg,
                    initialGg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final KnownBiasTurntableGyroscopeCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownBiasTurntableGyroscopeCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateEnd++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
    }

    private void checkLocked(final KnownBiasTurntableGyroscopeCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBiasX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBiasY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBiasZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBiasX(null));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBiasY(null));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBiasZ(null));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBias(
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBias(
                null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setBias((AngularSpeedTriad) null));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBias((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerBias((Matrix) null));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerSx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerSy(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerSz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerMxy(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerMxz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerMyx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerMyz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerMzx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerMzy(0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerScalingFactors(
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerCrossCouplingErrors(
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerMa(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasX(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasY(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasZ(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasCoordinates(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasCoordinates(null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setInitialSx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialSy(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialSz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMxy(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMxz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMyx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMyz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMzx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMzy(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialScalingFactors(
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setBias((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setBias((Matrix) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialMg(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialGg(null));
        assertThrows(LockedException.class, () -> calibrator.setTurntableRotationRate(0.0));
        assertThrows(LockedException.class, () -> calibrator.setTurntableRotationRate(null));
        assertThrows(LockedException.class, () -> calibrator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> calibrator.setTimeInterval(null));
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setPosition((ECEFPosition) null));
        assertThrows(LockedException.class, () -> calibrator.setPosition((NEDPosition) null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setGDependentCrossBiasesEstimated(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix mg, final Matrix gg, final KnownBiasTurntableGyroscopeCalibrator calibrator) {

        assertEquals(mg.getElementAt(0, 0), calibrator.getEstimatedSx(), 0.0);
        assertEquals(mg.getElementAt(1, 1), calibrator.getEstimatedSy(), 0.0);
        assertEquals(mg.getElementAt(2, 2), calibrator.getEstimatedSz(), 0.0);
        assertEquals(mg.getElementAt(0, 1), calibrator.getEstimatedMxy(), 0.0);
        assertEquals(mg.getElementAt(0, 2), calibrator.getEstimatedMxz(), 0.0);
        assertEquals(mg.getElementAt(1, 0), calibrator.getEstimatedMyx(), 0.0);
        assertEquals(mg.getElementAt(1, 2), calibrator.getEstimatedMyz(), 0.0);
        assertEquals(mg.getElementAt(2, 0), calibrator.getEstimatedMzx(), 0.0);
        assertEquals(mg.getElementAt(2, 1), calibrator.getEstimatedMzy(), 0.0);

        assertEquals(gg, calibrator.getEstimatedGg());
    }

    private static void checkCommonAxisAndGDependantCrossBiasesCovariance(final Matrix covariance) {
        assertEquals(18, covariance.getRows());
        assertEquals(18, covariance.getColumns());

        for (var j = 0; j < 18; j++) {
            final var colIsZero = j == 5 || j == 7 || j == 8;
            for (var i = 0; i < 18; i++) {
                final var rowIsZero = i == 5 || i == 7 || i == 8;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralAndGDependantCrossBiasesCovariance(final Matrix covariance) {
        assertEquals(18, covariance.getRows());
        assertEquals(18, covariance.getColumns());

        for (var i = 0; i < 18; i++) {
            assertNotEquals(0.0, covariance.getElementAt(i, i));
        }
    }

    private static void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(18, covariance.getRows());
        assertEquals(18, covariance.getColumns());

        for (var j = 0; j < 18; j++) {
            final var colIsZero = j == 5 || j > 6;
            for (var i = 0; i < 18; i++) {
                final var rowIsZero = i == 5 || i > 6;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(18, covariance.getRows());
        assertEquals(18, covariance.getColumns());

        for (var j = 0; j < 18; j++) {
            final var colIsZero = j > 8;
            for (var i = 0; i < 18; i++) {
                final var rowIsZero = i > 8;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
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

    private static Matrix generateCommonAxisMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGeneralMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                -300e-6, -300e-6, -150e-6,
                250e-6, -150e-6, -350e-6
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

    private static double getTurntableRotationRate() {
        final var randomizer = new UniformRandomizer();
        return Math.toRadians(randomizer.nextDouble(MIN_ROTATION_RATE_DEGREES_PER_SECOND,
                MAX_ROTATION_RATE_DEGREES_PER_SECOND));
    }
}
