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
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
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
import com.irurueta.navigation.inertial.BodyKinematics;
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
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class TurntableGyroscopeCalibratorTest implements TurntableGyroscopeCalibratorListener {

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

    private int mCalibrateStart;
    private int mCalibrateEnd;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(0.0, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(0.0, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(0.0, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(Constants.EARTH_ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(Constants.EARTH_ROTATION_RATE, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(TurntableGyroscopeCalibrator.DEFAULT_TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(TurntableGyroscopeCalibrator.DEFAULT_TIME_INTERVAL, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
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
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(timeInterval, calibrator.getTimeInterval(), 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(timeInterval, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, m6));
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, bg, mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, m6, this));
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, m4));
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, bias, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, m4, this));
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, new double[1], mg, gg, accelerometerBias, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, m1, gg, accelerometerBias, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, m2, gg, accelerometerBias, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, m3, accelerometerBias, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, m4, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, m6));
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, new double[1], mg, gg, accelerometerBias, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, m1, gg, accelerometerBias, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, m2, gg, accelerometerBias, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, m3, accelerometerBias, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, m4, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, m6, this));
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bg, mg, gg, ba, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, m10));
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, bg, mg, gg, ba, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, m10, this));
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, false, false, bg, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, true, false, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m6));
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, false, false, bg, mg, gg,
                this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, true, false, bg, mg, gg,
                this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, m1, mg, gg,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, m2, mg, gg,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m3, gg,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m4, gg,
                this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m5,
                this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bg, mg,
                m6, this));
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, false, false, bias, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, 0.0, measurements, true, false,
                bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, new double[1], mg,
                gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m4));
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, false, false, bias, mg, gg,
                this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, true, false, bias, mg, gg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, new double[1], mg,
                gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false, bias,
                m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m2, gg,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m3,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m4,
                this));
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, false, false, bias, mg, gg,
                accelerometerBias, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, 0.0,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, true, false, bias, mg, gg,
                accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                rotationRate, timeInterval, measurements, true, false,
                new double[1], mg, gg, accelerometerBias, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m1, gg,
                accelerometerBias, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m2, gg,
                accelerometerBias, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m3,
                accelerometerBias, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg,
                m4, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, m6));
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, false, false, bias, mg, gg,
                accelerometerBias, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, true, false, bias, mg, gg,
                accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, new double[1], mg,
                gg, accelerometerBias, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m1, gg,
                accelerometerBias, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m2, gg,
                accelerometerBias, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m3,
                accelerometerBias, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m4,
                accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, m6, this));
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, false, false, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, ba, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, true, false, bg, mg, gg, ba,
                ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, ba,
                m10));
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, false, false, bg, mg, gg, ba, ma,
                this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, ba, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                0.0, measurements, true, false, bg, mg, gg, ba, ma,
                this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, m1, mg, gg, ba, ma,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, m2, mg, gg, ba, ma,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m3, gg, ba, ma,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m4, gg, ba, ma,
                this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m5, ba, ma,
                this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m6, ba, ma,
                this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, m7, ma,
                this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, m8, ma,
                this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, ba, m9,
                this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, ba, m10,
                this));
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, m6));
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, bg, mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, m6, this));
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, m4));
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, m4, this));
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, new double[1], mg, gg, accelerometerBias, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, m1, gg, accelerometerBias, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, m2, gg, accelerometerBias, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, m3, accelerometerBias, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, m4, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, m6));
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, new double[1], mg, gg, accelerometerBias, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, m1, gg, accelerometerBias, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, m2, gg, accelerometerBias, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, m3, accelerometerBias, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, m4, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bias, mg, gg, accelerometerBias, m6, this));
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bg, mg, gg, ba, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, m10));
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, bg, mg, gg, ba, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, bg, mg, gg, ba, m10, this));
    }

    @Test
    public void testConstructor26() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, false, false, bg, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, true, false, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                rotationRate, timeInterval, measurements, true, false, bg, m4,
                gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m6));
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, false, false, bg, mg, gg,
                this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, true, false, bg, mg, gg,
                this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, m1, mg, gg,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, m2, mg, gg,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m3, gg,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m4, gg,
                this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m5,
                this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m6,
                this));
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, false, false, bias, mg, gg);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, true, false, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, new double[1], mg,
                gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m4));
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, false, false, bias, mg, gg,
                this);

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], accelerometerBias1, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, true, false, bias, mg, gg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, new double[1], mg,
                gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m1, gg,
                this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m2, gg,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m3,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m4,
                this));
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, false, false, bias, mg, gg,
                accelerometerBias, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, true, false, bias, mg, gg,
                accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, new double[1], mg,
                gg, accelerometerBias, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m1, gg,
                accelerometerBias, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m2, gg,
                accelerometerBias, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m3,
                accelerometerBias, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m4,
                accelerometerBias, ma));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, m6));
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, false, false, bias, mg, gg,
                accelerometerBias, ma, this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bias, mg, gg, accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                0.0, measurements, true, false, bias, mg, gg,
                accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, new double[1], mg,
                gg, accelerometerBias, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m1, gg,
                accelerometerBias, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, m2, gg,
                accelerometerBias, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m3,
                accelerometerBias, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, m4,
                accelerometerBias, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bias, mg, gg,
                accelerometerBias, m6, this));
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, false, false, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, ba,
                m10));
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, false, false, bg, mg, gg, ba, ma,
                this);

        // check default values
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(bax, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(bay, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(biasTriad2);
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
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);
        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition,
                0.0, timeInterval, measurements, true,
                false, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, m1, mg, gg, ba, ma,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, m2, mg, gg, ba, ma,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m3, gg, ba, ma,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, m4, gg, ba, ma,
                this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m5, ba, ma,
                this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, m6, ba, ma,
                this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, m7, ma,
                this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, m8, ma,
                this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, ba,
                m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                timeInterval, measurements, true, false, bg, mg, gg, ba, m10,
                this));
    }

    @Test
    public void testGetSetAccelerometerBiasX() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);

        calibrator.setAccelerometerBiasX(bax);

        // check
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasY() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);

        calibrator.setAccelerometerBiasY(bay);

        // check
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasZ() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);

        calibrator.setAccelerometerBiasZ(baz);

        // check
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasXAsAcceleration() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        final Acceleration acceleration1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final Acceleration acceleration2 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasX(acceleration2);

        // check
        final Acceleration acceleration3 = calibrator.getAccelerometerBiasXAsAcceleration();
        final Acceleration acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    public void testGetSetAccelerometerBiasYAsAcceleration() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        final Acceleration acceleration1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);
        final Acceleration acceleration2 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasY(acceleration2);

        // check
        final Acceleration acceleration3 = calibrator.getAccelerometerBiasYAsAcceleration();
        final Acceleration acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    public void testGetSetAccelerometerBiasZAsAcceleration() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        final Acceleration acceleration1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);
        final Acceleration acceleration2 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasZ(acceleration2);

        // check
        final Acceleration acceleration3 = calibrator.getAccelerometerBiasZAsAcceleration();
        final Acceleration acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    public void testSetAccelerometerBias1() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);


        // set new values
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        calibrator.setAccelerometerBias(bax, bay, baz);

        // check
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
    }

    @Test
    public void testSetAccelerometerBias2() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);

        // set new values
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final Acceleration accelerationX = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setAccelerometerBias(accelerationX, accelerationY, accelerationZ);

        // check
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasArray() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertArrayEquals(calibrator.getAccelerometerBias(), new double[3], 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double[] bias = ba.getBuffer();

        calibrator.setAccelerometerBias(bias);

        // check
        final double[] bias1 = calibrator.getAccelerometerBias();
        final double[] bias2 = new double[3];
        calibrator.getAccelerometerBias(bias2);

        assertArrayEquals(bias, bias1, 0.0);
        assertArrayEquals(bias, bias2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccelerometerBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setAccelerometerBias(new double[1]));
    }

    @Test
    public void testGetSetAccelerometerBiasAsMatrix() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getAccelerometerBiasAsMatrix());

        // set new value
        final Matrix ba = generateBa();
        calibrator.setAccelerometerBias(ba);

        // check
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        final Matrix ba2 = new Matrix(3, 1);
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
    public void testGetSetAccelerometerSx() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double asx = ma.getElementAt(0, 0);

        calibrator.setAccelerometerSx(asx);

        // check
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerSy() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double asy = ma.getElementAt(1, 1);

        calibrator.setAccelerometerSy(asy);

        // check
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerSz() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double asz = ma.getElementAt(2, 2);

        calibrator.setAccelerometerSz(asz);

        // check
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMxy() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amxy = ma.getElementAt(0, 1);

        calibrator.setAccelerometerMxy(amxy);

        // check
        assertEquals(amxy, calibrator.getAccelerometerMxy(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMxz() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amxz = ma.getElementAt(0, 2);

        calibrator.setAccelerometerMxz(amxz);

        // check
        assertEquals(amxz, calibrator.getAccelerometerMxz(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMyx() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amyx = ma.getElementAt(1, 0);

        calibrator.setAccelerometerMyx(amyx);

        // check
        assertEquals(amyx, calibrator.getAccelerometerMyx(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMyz() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amyz = ma.getElementAt(1, 2);

        calibrator.setAccelerometerMyz(amyz);

        // check
        assertEquals(amyz, calibrator.getAccelerometerMyz(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMzx() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amzx = ma.getElementAt(2, 0);

        calibrator.setAccelerometerMzx(amzx);

        // check
        assertEquals(amzx, calibrator.getAccelerometerMzx(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMzy() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMzy(amzy);

        // check
        assertEquals(amzy, calibrator.getAccelerometerMzy(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerScalingFactors() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);

        // set new values
        final Matrix ma = generateMa();
        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);

        calibrator.setAccelerometerScalingFactors(asx, asy, asz);

        // check
        assertEquals(asx, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(asy, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(asz, calibrator.getAccelerometerSz(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerCrossCouplingErrors() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new values
        final Matrix ma = generateMa();
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

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
    public void testGetSetAccelerometerScalingFactorsAndCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

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
        final Matrix ma = generateMa();
        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(asx, asy, asz, amxy, amxz, amyx,
                amyz, amzx, amzy);

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
    public void testGetSetAccelerometerMa() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

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
        final Matrix ma = generateMa();
        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMa(ma);

        // check
        final Matrix ma1 = calibrator.getAccelerometerMa();
        final Matrix ma2 = new Matrix(3, 3);
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
    public void testGetSetInitialBiasX() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator =
                new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);

        calibrator.setInitialBiasX(bx);

        // check
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
    }

    @Test
    public void testGetSetInitialBiasY() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double by = bg.getElementAtIndex(1);

        calibrator.setInitialBiasY(by);

        // check
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
    }

    @Test
    public void testGetSetInitialBiasZ() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bz = bg.getElementAtIndex(2);

        calibrator.setInitialBiasZ(bz);

        // check
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedX() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        final AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);
        final AngularSpeed angularSpeed2 = new AngularSpeed(bx, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBiasX(angularSpeed2);

        // check
        final AngularSpeed angularSpeed3 = calibrator.getInitialBiasAngularSpeedX();
        final AngularSpeed angularSpeed4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedY() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        final AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double by = bg.getElementAtIndex(1);
        final AngularSpeed angularSpeed2 = new AngularSpeed(by, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBiasY(angularSpeed2);

        // check
        final AngularSpeed angularSpeed3 = calibrator.getInitialBiasAngularSpeedY();
        final AngularSpeed angularSpeed4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedZ() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        final AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bz = bg.getElementAtIndex(2);
        final AngularSpeed angularSpeed2 = new AngularSpeed(bz, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBiasZ(angularSpeed2);

        // check
        final AngularSpeed angularSpeed3 = calibrator.getInitialBiasAngularSpeedZ();
        final AngularSpeed angularSpeed4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    public void testSetInitialBias1() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);
        final double by = bg.getElementAtIndex(1);
        final double bz = bg.getElementAtIndex(2);

        calibrator.setInitialBias(bx, by, bz);

        // check
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testSetInitialBias2() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);
        final double by = bg.getElementAtIndex(1);
        final double bz = bg.getElementAtIndex(2);

        final AngularSpeed angularSpeedX = new AngularSpeed(bx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(by, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(bz, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBias(angularSpeedX, angularSpeedY, angularSpeedZ);

        // check
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testGetSetInitialBiasAsTriad() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        final AngularSpeedTriad triad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final AngularSpeedTriad triad2 = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);
        calibrator.setInitialBias(triad2);

        // check
        final AngularSpeedTriad triad3 = calibrator.getInitialBiasAsTriad();
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double sx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(sx);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double sy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(sy);

        // check
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double sz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(sz);

        // check
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double mxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double mxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double myx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(myx);

        // check
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double myz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(myz);

        // check
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double mzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final Matrix mg = generateCommonAxisMg();
        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testSetInitialCrossCouplingErrors() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final Matrix mg = generateCommonAxisMg();
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

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
    public void testSetInitialScalingFactorsAndCrossCouplingErrors() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

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
        final Matrix mg = generateCommonAxisMg();
        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

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
    public void testGetSetInitialBiasArray() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertArrayEquals(new double[3], calibrator.getInitialBias(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double[] bias = bg.getBuffer();

        calibrator.setInitialBias(bias);

        // check
        final double[] bias1 = calibrator.getInitialBias();
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);

        assertArrayEquals(bias, bias1, 0.0);
        assertArrayEquals(bias, bias2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(new double[1]));
    }

    @Test
    public void testGetSetInitialBiasAsMatrix() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getInitialBiasAsMatrix());

        // set new value
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);
        final double by = bg.getElementAtIndex(1);
        final double bz = bg.getElementAtIndex(2);

        calibrator.setInitialBias(bg);

        // check
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);

        assertEquals(bg, bg1);
        assertEquals(bg, bg2);

        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBiasAsMatrix(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBiasAsMatrix(m2));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(m4));
    }

    @Test
    public void testGetSetInitialMg() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

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
        final Matrix mg = generateCommonAxisMg();
        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMg(mg);

        // check
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
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
    public void testGetSetInitialGg() throws WrongSizeException, LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(new Matrix(3, 3), calibrator.getInitialGg());

        // set new value
        final Matrix gg = generateGg();
        calibrator.setInitialGg(gg);

        // check
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
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
    public void testGetSetTurntableRotationRate() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(Constants.EARTH_ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);

        // set new value
        final double rotationRate = getTurntableRotationRate();
        calibrator.setTurntableRotationRate(rotationRate);

        // check
        assertEquals(rotationRate, calibrator.getTurntableRotationRate(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setTurntableRotationRate(0.0));
    }

    @Test
    public void testGetSetTurntableRotationRateAsAngularSpeed() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        final AngularSpeed rotationRate1 = calibrator.getTurntableRotationRateAsAngularSpeed();
        assertEquals(Constants.EARTH_ROTATION_RATE, rotationRate1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, rotationRate1.getUnit());

        // set new value
        final double rotationRate = getTurntableRotationRate();
        final AngularSpeed rotationRate2 = new AngularSpeed(rotationRate, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setTurntableRotationRate(rotationRate2);

        // check
        final AngularSpeed rotation3 = calibrator.getTurntableRotationRateAsAngularSpeed();
        final AngularSpeed rotation4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotation4);

        assertEquals(rotation3, rotation4);

        // Force IllegalArgumentException
        final var w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setTurntableRotationRate(w));
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final Collection<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testGetSetEcefPosition() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getEcefPosition());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        calibrator.setPosition(ecefPosition);

        // check
        assertSame(ecefPosition, calibrator.getEcefPosition());
    }

    @Test
    public void testGetSetNedPosition() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        calibrator.setPosition(nedPosition);

        // check
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition position2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(position2));
        assertTrue(nedPosition.equals(position2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertFalse(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testIsSetGDependentCrossBiasesEstimated() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testGetMinimumRequiredMeasurementsOrSequences() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertTrue(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(true);

        // check
        assertEquals(22, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    public void testIsReady() throws LockedException {
        final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isReady());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());

        // set empty measurements
        final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
        calibrator.setMeasurements(measurements);

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements
        for (int i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS_AND_CROSS_BIASES; i++) {
            measurements.add(new StandardDeviationBodyKinematics());
        }

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = new Matrix(3, 3);
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final double accelNoiseRootPSD = 0.0;
            final double gyroNoiseRootPSD = 0.0;
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS; i++) {
                final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        timeInterval, ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics(
                        measuredKinematics, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                    timeInterval, measurements, true, false, bg, mg, gg,
                    this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesDisabledAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = new Matrix(3, 3);
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final double accelNoiseRootPSD = 0.0;
            final double gyroNoiseRootPSD = 0.0;
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_GENERAL; i++) {
                final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        timeInterval, ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics(
                        measuredKinematics, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                    timeInterval, measurements, false, false, bg, mg, gg,
                    this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < 3 * TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final double accelNoiseRootPSD = 0.0;
            final double gyroNoiseRootPSD = 0.0;
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS_AND_CROSS_BIASES; i++) {
                final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        timeInterval, ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics(
                        measuredKinematics, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                    timeInterval, measurements, true, true,
                    bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesEnabledAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final double accelNoiseRootPSD = 0.0;
            final double gyroNoiseRootPSD = 0.0;
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES; i++) {
                final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        timeInterval, ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics(
                        measuredKinematics, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                    timeInterval, measurements, false, true,
                    bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledLargeNumberOfMeasurementsWithNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = new Matrix(3, 3);
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        timeInterval, ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics(
                        measuredKinematics, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final Matrix initialBg = new Matrix(3, 1);
            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                    timeInterval, measurements, true, false, initialBg,
                    initialMg, initialGg, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesDisabledLargeNumberOfMeasurementsWithNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = new Matrix(3, 3);
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        timeInterval, ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics(
                        measuredKinematics, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final Matrix initialBg = new Matrix(3, 1);
            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                    timeInterval, measurements, false, false, initialBg,
                    initialMg, initialGg, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledLargeNumberOfMeasurementsWithNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        timeInterval, ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics(
                        measuredKinematics, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final Matrix initialBg = new Matrix(3, 1);
            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                    timeInterval, measurements, true, true, initialBg,
                    initialMg, initialGg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesEnabledLargeNumberOfMeasurementsWithNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(roll1, pitch1, yaw1,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 = new CoordinateTransformation(rot2.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        timeInterval, ecefFrame2, ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(timeInterval,
                        trueKinematics, errors, random);

                final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics(
                        measuredKinematics, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final Matrix initialBg = new Matrix(3, 1);
            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final TurntableGyroscopeCalibrator calibrator = new TurntableGyroscopeCalibrator(nedPosition, rotationRate,
                    timeInterval, measurements, false, true, initialBg,
                    initialMg, initialGg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final TurntableGyroscopeCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final TurntableGyroscopeCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateEnd++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(final TurntableGyroscopeCalibrator calibrator) {
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
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasX(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasY(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasZ(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias(
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias(
                null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((AngularSpeedTriad) null));
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
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((Matrix) null));
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
            final Matrix bg, final Matrix mg, final Matrix gg, final TurntableGyroscopeCalibrator calibrator)
            throws WrongSizeException {

        final double[] estimatedBiases = calibrator.getEstimatedBiases();
        assertArrayEquals(bg.getBuffer(), estimatedBiases, 0.0);

        final double[] estimatedBiases2 = new double[3];
        calibrator.getEstimatedBiases(estimatedBiases2);
        assertArrayEquals(estimatedBiases, estimatedBiases2, 0.0);

        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getEstimatedBiasesAsMatrix(bg2);

        assertEquals(bg, bg2);

        assertEquals(bg.getElementAtIndex(0), calibrator.getEstimatedBiasX(), 0.0);
        assertEquals(bg.getElementAtIndex(1), calibrator.getEstimatedBiasY(), 0.0);
        assertEquals(bg.getElementAtIndex(2), calibrator.getEstimatedBiasZ(), 0.0);

        final AngularSpeed bgx1 = calibrator.getEstimatedBiasAngularSpeedX();
        final AngularSpeed bgx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        assertEquals(bgx1.getValue().doubleValue(), calibrator.getEstimatedBiasX(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        final AngularSpeed bgy1 = calibrator.getEstimatedBiasAngularSpeedY();
        final AngularSpeed bgy2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        assertEquals(bgy1.getValue().doubleValue(), calibrator.getEstimatedBiasY(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        final AngularSpeed bgz1 = calibrator.getEstimatedBiasAngularSpeedZ();
        final AngularSpeed bgz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
        assertEquals(bgz1.getValue().doubleValue(), calibrator.getEstimatedBiasZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

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

        assertCovariance(calibrator);
    }

    private static void assertCovariance(final TurntableGyroscopeCalibrator calibrator) {
        assertNotNull(calibrator.getEstimatedBiasXVariance());

        assertNotNull(calibrator.getEstimatedBiasXVariance());
        assertNotNull(calibrator.getEstimatedBiasXStandardDeviation());
        final AngularSpeed stdBx1 = calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed();
        assertNotNull(stdBx1);
        final AngularSpeed stdBx2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(stdBx2));
        assertEquals(stdBx1, stdBx2);

        assertNotNull(calibrator.getEstimatedBiasYVariance());
        assertNotNull(calibrator.getEstimatedBiasYStandardDeviation());
        final AngularSpeed stdBy1 = calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed();
        assertNotNull(stdBy1);
        final AngularSpeed stdBy2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(stdBy2));
        assertEquals(stdBy1, stdBy2);

        assertNotNull(calibrator.getEstimatedBiasZVariance());
        assertNotNull(calibrator.getEstimatedBiasZStandardDeviation());
        final AngularSpeed stdBz1 = calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed();
        assertNotNull(stdBz1);
        final AngularSpeed stdBz2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(stdBz2));
        assertEquals(stdBz1, stdBz2);

        final AngularSpeedTriad std1 = calibrator.getEstimatedBiasStandardDeviation();
        assertEquals(std1.getValueX(), calibrator.getEstimatedBiasXStandardDeviation(), 0.0);
        assertEquals(std1.getValueY(), calibrator.getEstimatedBiasYStandardDeviation(), 0.0);
        assertEquals(std1.getValueZ(), calibrator.getEstimatedBiasZStandardDeviation(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final AngularSpeedTriad std2 = new AngularSpeedTriad();
        calibrator.getEstimatedBiasStandardDeviation(std2);

        final double avgStd = (calibrator.getEstimatedBiasXStandardDeviation() +
                calibrator.getEstimatedBiasYStandardDeviation() +
                calibrator.getEstimatedBiasZStandardDeviation()) / 3.0;
        assertEquals(avgStd, calibrator.getEstimatedBiasStandardDeviationAverage(), 0.0);
        final AngularSpeed avg1 = calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed();
        assertEquals(avgStd, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avg1.getUnit());
        final AngularSpeed avg2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(avg2);
        assertEquals(avg1, avg2);

        assertEquals(std1.getNorm(), calibrator.getEstimatedBiasStandardDeviationNorm(), ABSOLUTE_ERROR);
        final AngularSpeed norm1 = calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed();
        assertEquals(std1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        final AngularSpeed norm2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(norm2);
        assertEquals(norm1, norm2);
    }

    private static void checkCommonAxisAndGDependantCrossBiasesCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (int j = 0; j < 21; j++) {
            final boolean colIsZero = j == 8 || j == 10 || j == 11;
            for (int i = 0; i < 21; i++) {
                final boolean rowIsZero = i == 8 || i == 10 || i == 11;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralAndGDependantCrossBiasesCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (int i = 0; i < 21; i++) {
            assertNotEquals(covariance.getElementAt(i, i), 0.0);
        }
    }

    private static void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (int j = 0; j < 21; j++) {
            final boolean colIsZero = j == 8 || j > 9;
            for (int i = 0; i < 21; i++) {
                final boolean rowIsZero = i == 8 || i > 9;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (int j = 0; j < 21; j++) {
            final boolean colIsZero = j > 11;
            for (int i = 0; i < 21; i++) {
                final boolean rowIsZero = i > 11;
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
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateCommonAxisMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGeneralMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                -300e-6, -300e-6, -150e-6,
                250e-6, -150e-6, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        final double tmp = DEG_TO_RAD / (3600 * 9.80665);
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
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        return Math.toRadians(randomizer.nextDouble(MIN_ROTATION_RATE_DEGREES_PER_SECOND,
                MAX_ROTATION_RATE_DEGREES_PER_SECOND));
    }
}
