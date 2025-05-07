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
package com.irurueta.navigation.inertial.calibration.accelerometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorTest implements
        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener {

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

    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int TIMES = 100;

    private int mCalibrateStart;
    private int mCalibrateEnd;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[3];
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(this);

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[3];
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[3];
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                this);

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[3];
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(true);

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[3];
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                true, this);

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[3];
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true);

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, this);

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[3];
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                true);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                true, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, true);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, true, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor17() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX, baY, baZ);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor18() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX, baY, baZ,
                this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                baX, baY, baZ);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                baX, baY, baZ, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor21() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX, baY, baZ,
                true);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor22() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX, baY, baZ,
                true, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor23() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                baX, baY, baZ, true);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor24() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                baX, baY, baZ, true, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor25() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor26() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor27() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor28() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor29() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor30() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, biasX, biasY, biasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor31() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, biasX, biasY, biasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor32() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX, baY, baZ,
                initialSx, initialSy, initialSz);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor33() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX, baY, baZ,
                initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor34() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                baX, baY, baZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor35() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                baX, baY, baZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor36() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX, baY, baZ,
                true, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor37() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                baX, baY, baZ, true, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor38() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                baX, baY, baZ, true, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor39() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor40() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor41() throws WrongSizeException {

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                true, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor42() throws WrongSizeException {

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                true, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor43() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, biasX, biasY, biasZ, true, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor44() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, biasX, biasY, biasZ, true, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor45() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                baX, baY, baZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor46() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                baX, baY, baZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor47() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, baX, baY, baZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor48() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, baX, baY, baZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor49() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                baX, baY, baZ, true, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor50() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX, baY, baZ,
                true, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor51() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, baX, baY, baZ, true, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor52() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var baX = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baY = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baZ = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, baX, baY, baZ, true, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor53() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var bias = new double[]{biasX, biasY, biasZ};

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(bias);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(new double[1]));
    }

    @Test
    void testConstructor54() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var bias = new double[]{biasX, biasY, biasZ};

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(bias, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(new double[1], this));
    }

    @Test
    void testConstructor55() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var bias = new double[]{biasX, biasY, biasZ};

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bias);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, new double[1]));
    }

    @Test
    void testConstructor56() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var bias = new double[]{biasX, biasY, biasZ};

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bias,
                this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, new double[1],
                        this));
    }

    @Test
    void testConstructor57() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var bias = new double[]{biasX, biasY, biasZ};

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(bias,
                true);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(new double[1],
                        true));
    }

    @Test
    void testConstructor58() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var bias = new double[]{biasX, biasY, biasZ};

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                bias, true, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(new double[1], true));
    }

    @Test
    void testConstructor59() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var bias = new double[]{biasX, biasY, biasZ};

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bias,
                true);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(new double[1],
                        true));
    }

    @Test
    void testConstructor60() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var bias = new double[]{biasX, biasY, biasZ};

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bias,
                true, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(new double[1], true));
    }

    @Test
    void testConstructor61() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m2));
    }

    @Test
    void testConstructor62() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m2, this));
    }

    @Test
    void testConstructor63() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m2));
    }

    @Test
    void testConstructor64() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m2));
    }

    @Test
    void testConstructor65() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba,
                true);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m1, true));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m2, true));
    }

    @Test
    void testConstructor66() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba,
                true, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m1, true,
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () ->
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m2, true,
                        this));
    }

    @Test
    void testConstructor67() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                true);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m1,
                        true));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m2,
                        true));
    }

    @Test
    void testConstructor68() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                true, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m1,
                        true, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m2,
                        true, this));
    }

    @Test
    void testConstructor69() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, ma);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, m4));
    }

    @Test
    void testConstructor70() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, ma, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, m4));
    }

    @Test
    void testConstructor71() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba, ma);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba, m4));
    }

    @Test
    void testConstructor72() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba, ma,
                this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m1, ma,
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m2, ma,
                        this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba, m3,
                        this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba, m4,
                        this));
    }

    @Test
    void testConstructor73() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba,
                true, ma);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m1, true, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m2, true, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, true, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, true, m4));
    }

    @Test
    void testConstructor74() throws WrongSizeException {
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba,
                true, ma, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m1, true, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(m2, true, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, true, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba, true, m4));
    }

    @Test
    void testConstructor75() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                true, ma);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(bias1, calibrator.getBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m1,
                        true, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m2,
                        true, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                        true, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                        true, m4));
    }

    @Test
    void testConstructor76() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                true, ma, this);

        // check default values
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        var acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = new double[]{biasX, biasY, biasZ};
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m1,
                        true, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, m2,
                        true, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                        true, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                        true, m4));
    }

    @Test
    void testGetSetBiasX() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasX(), 0.0);

        // set new value
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);

        calibrator.setBiasX(biasX);

        // check
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
    }

    @Test
    void testGetSetBiasY() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasY(), 0.0);

        // set new value
        final var ba = generateBa();
        final var biasY = ba.getElementAtIndex(1);

        calibrator.setBiasY(biasY);

        // check
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
    }

    @Test
    void testGetSetBiasZ() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new value
        final var ba = generateBa();
        final var biasZ = ba.getElementAtIndex(2);

        calibrator.setBiasZ(biasZ);

        // check
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
    }

    @Test
    void testGetSetBiasXAsAcceleration() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasX1 = calibrator.getBiasXAsAcceleration();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasX1.getUnit());

        // set new value
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasX2 = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasX(biasX2);

        // check
        final var biasX3 = calibrator.getBiasXAsAcceleration();
        final var biasX4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(biasX4);

        assertEquals(biasX2, biasX3);
        assertEquals(biasX2, biasX4);
    }

    @Test
    void testGetSetBiasYAsAcceleration() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasY1 = calibrator.getBiasYAsAcceleration();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasY1.getUnit());

        // set new value
        final var ba = generateBa();
        final var biasY = ba.getElementAtIndex(1);
        final var biasY2 = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasY(biasY2);

        // check
        final var biasY3 = calibrator.getBiasYAsAcceleration();
        final var biasY4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(biasY4);

        assertEquals(biasY2, biasY3);
        assertEquals(biasY2, biasY4);
    }

    @Test
    void testGetSetBiasZAsAcceleration() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasZ1 = calibrator.getBiasZAsAcceleration();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasZ1.getUnit());

        // set new value
        final var ba = generateBa();
        final var biasZ = ba.getElementAtIndex(2);
        final var biasZ2 = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasZ(biasZ2);

        // check
        final var biasZ3 = calibrator.getBiasZAsAcceleration();
        final var biasZ4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(biasZ4);

        assertEquals(biasZ2, biasZ3);
        assertEquals(biasZ2, biasZ4);
    }

    @Test
    void testSetBiasCoordinates1() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new values
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        calibrator.setBiasCoordinates(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
    }

    @Test
    void testSetBiasCoordinates2() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new values
        final var ba = generateBa();
        final var biasX = new Acceleration(ba.getElementAtIndex(0), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var biasY = new Acceleration(ba.getElementAtIndex(1), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var biasZ = new Acceleration(ba.getElementAtIndex(2), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasCoordinates(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getBiasXAsAcceleration());
        assertEquals(biasY, calibrator.getBiasYAsAcceleration());
        assertEquals(biasZ, calibrator.getBiasZAsAcceleration());
    }

    @Test
    void testGetSetBiasAsTriad() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        final var triad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        // set new values
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var triad2 = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasX, biasY, biasZ);
        calibrator.setBias(triad2);

        // check
        final var triad3 = calibrator.getBiasAsTriad();
        final var triad4 = new AccelerationTriad();
        calibrator.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);

        calibrator.setInitialSx(initialSx);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialSy = ma.getElementAt(1, 1);

        calibrator.setInitialSy(initialSy);

        // check
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialSz = ma.getElementAt(2, 2);

        calibrator.setInitialSz(initialSz);

        // check
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialMxy = ma.getElementAt(0, 1);

        calibrator.setInitialMxy(initialMxy);

        // check
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialMxz = ma.getElementAt(0, 2);

        calibrator.setInitialMxz(initialMxz);

        // check
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialMyx = ma.getElementAt(1, 0);

        calibrator.setInitialMyx(initialMyx);

        // check
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialMyz = ma.getElementAt(1, 2);

        calibrator.setInitialMyz(initialMyz);

        // check
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialMzx = ma.getElementAt(2, 0);

        calibrator.setInitialMzx(initialMzx);

        // check
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new value
        final var ma = generateMaGeneral();
        final var initialMzy = ma.getElementAt(2, 1);

        calibrator.setInitialMzy(initialMzy);

        // check
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testSetInitialScalingFactorsAndCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testGetSetBiasAsArray() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        final var bias1 = calibrator.getBias();
        final var bias2 = new double[BodyKinematics.COMPONENTS];
        calibrator.getBias(bias2);

        assertArrayEquals(new double[BodyKinematics.COMPONENTS], bias1, 0.0);
        assertArrayEquals(bias1, bias2, 0.0);

        // set new values
        final var ba = generateBa();
        final var bias3 = ba.getBuffer();

        calibrator.setBias(bias3);

        // check
        final var bias4 = new double[BodyKinematics.COMPONENTS];
        calibrator.getBias(bias4);
        final var bias5 = calibrator.getBias();

        assertArrayEquals(bias3, bias4, 0.0);
        assertArrayEquals(bias3, bias5, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setBias(new double[1]));
    }

    @Test
    void testGetSetBiasAsMatrix() throws LockedException, WrongSizeException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        final var bias1 = calibrator.getBiasAsMatrix();
        final var bias2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        calibrator.getBiasAsMatrix(bias2);

        assertEquals(0.0, bias1.getElementAtIndex(0), 0.0);
        assertEquals(0.0, bias1.getElementAtIndex(1), 0.0);
        assertEquals(0.0, bias1.getElementAtIndex(2), 0.0);
        assertEquals(bias1, bias2);

        // set new value
        final var bias3 = generateBa();
        calibrator.setBias(bias3);

        // check
        final var bias4 = calibrator.getBiasAsMatrix();
        final var bias5 = new Matrix(BodyKinematics.COMPONENTS, 1);
        calibrator.getBiasAsMatrix(bias5);

        assertEquals(bias3, bias4);
        assertEquals(bias3, bias5);

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
    void testGetSetInitialMa() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var ma1 = calibrator.getInitialMa();
        final var ma2 = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        calibrator.getInitialMa(ma2);

        assertEquals(new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS), ma1);
        assertEquals(ma1, ma2);

        // set new value
        final var ma3 = generateMaGeneral();
        calibrator.setInitialMa(ma3);

        // check
        final var ma4 = calibrator.getInitialMa();
        final var ma5 = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        calibrator.getInitialMa(ma5);

        assertEquals(ma3, ma4);
        assertEquals(ma3, ma5);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialMa(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialMa(m2));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialMa(m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialMa(m4));
    }

    @Test
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testIsReady() throws LockedException {
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isReady());
        assertNull(calibrator.getMeasurements());

        // set enough measurements
        calibrator.setMeasurements(Arrays.asList(
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics()));

        // check
        assertTrue(calibrator.isReady());
        assertNotNull(calibrator.getMeasurements());
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaGeneral();
        final var mg = generateMg();
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

        final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
        for (var i = 0; i < KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            // apply known calibration parameters to distort ground-truth and generate a
            // measured kinematics sample
            final var random = new Random();
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                    errors, random);

            final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                    TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                false, ma, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(0, mCalibrateStart);
        assertEquals(0, mCalibrateEnd);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(1, mCalibrateStart);
        assertEquals(1, mCalibrateEnd);

        final var estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMa, calibrator);

        assertNotNull(calibrator.getEstimatedCovariance());
        checkGeneralCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedMse() >= 0.0);
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseLargeNumberOfMeasurements() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMaGeneral();
            final var mg = generateMg();
            final var gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
            for (var i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {

                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefFrame, ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var random = new Random();
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                        errors, random);

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have a large number of measurements, there is no need to provide
            // an initial value because the default initial value (all zeros) will
            // typically converge close to the true solution
            final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                    false, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() > 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaGeneral();
        final var mg = generateMg();
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
        final var roll = 0.0;
        final var pitch = 0.0;
        final var yaw = 0.0;
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
        for (var i = 0; i < KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            // apply known calibration parameters to distort ground-truth and generate a
            // measured kinematics sample
            final var random = new Random();
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                    errors, random);

            final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                    TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                false, ma, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(0, mCalibrateStart);
        assertEquals(0, mCalibrateEnd);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(1, mCalibrateStart);
        assertEquals(1, mCalibrateEnd);

        final var estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ma.equals(estimatedMa, VERY_LARGE_ABSOLUTE_ERROR));

        assertNotNull(calibrator.getEstimatedCovariance());
        checkGeneralCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedMse() >= 0.0);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();
        // when using minimum number of measurements we must not add any noise so that
        // a solution is found. When adding more measurements, certain noise can be added
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var random = new Random();
        final var randomizer = new UniformRandomizer(random);
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
        for (var i = 0; i < KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            // apply known calibration parameters to distort ground-truth and generate a
            // measured kinematics sample
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                    errors, random);

            final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                    TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                true, ma, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(0, mCalibrateStart);
        assertEquals(0, mCalibrateEnd);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(1, mCalibrateStart);
        assertEquals(1, mCalibrateEnd);

        final var estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

        assertNotNull(calibrator.getEstimatedCovariance());
        checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedMse() >= 0.0);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseLargeNumberOfMeasurements()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, CalibrationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMaCommonAxis();
            final var mg = generateMg();
            final var gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
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

            final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
            for (var i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {

                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefFrame, ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var random = new Random();
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                        errors, random);

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have a large number of measurements, there is no need to provide
            // an initial value because the default initial value (all zeros) will
            // typically converge close to the true solution
            final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                    true, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);

            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() > 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
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
        final var roll = 0.0;
        final var pitch = 0.0;
        final var yaw = 0.0;
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
        for (var i = 0; i < KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            // apply known calibration parameters to distort ground-truth and generate a
            // measured kinematics sample
            final var random = new Random();
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random);

            final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                    TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final var calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, ba,
                true, ma, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(0, mCalibrateStart);
        assertEquals(0, mCalibrateEnd);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(1, mCalibrateStart);
        assertEquals(1, mCalibrateEnd);

        final var estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

        assertNotNull(calibrator.getEstimatedCovariance());
        checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedMse() >= 0.0);
    }

    @Override
    public void onCalibrateStart(final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateEnd++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setBiasX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasX(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasY(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasZ(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasCoordinates(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasCoordinates(null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setBias((AccelerationTriad) null));
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
        assertThrows(LockedException.class, () -> calibrator.setInitialMa(null));
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix ma, final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {

        assertEquals(ma.getElementAt(0, 0), calibrator.getEstimatedSx(), 0.0);
        assertEquals(ma.getElementAt(1, 1), calibrator.getEstimatedSy(), 0.0);
        assertEquals(ma.getElementAt(2, 2), calibrator.getEstimatedSz(), 0.0);
        assertEquals(ma.getElementAt(0, 1), calibrator.getEstimatedMxy(), 0.0);
        assertEquals(ma.getElementAt(0, 2), calibrator.getEstimatedMxz(), 0.0);
        assertEquals(ma.getElementAt(1, 0), calibrator.getEstimatedMyx(), 0.0);
        assertEquals(ma.getElementAt(1, 2), calibrator.getEstimatedMyz(), 0.0);
        assertEquals(ma.getElementAt(2, 0), calibrator.getEstimatedMzx(), 0.0);
        assertEquals(ma.getElementAt(2, 1), calibrator.getEstimatedMzy(), 0.0);
    }

    private static void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(9, covariance.getRows());
        assertEquals(9, covariance.getColumns());

        for (var j = 0; j < 9; j++) {
            final var colIsZero = j == 5 || j == 7 || j == 8;
            for (var i = 0; i < 9; i++) {
                final var rowIsZero = i == 5 || i == 7 || i == 8;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(9, covariance.getRows());
        assertEquals(9, covariance.getColumns());

        for (var i = 0; i < 9; i++) {
            assertNotEquals(0.0, covariance.getElementAt(i, i));
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

    private static Matrix generateMaGeneral() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMaCommonAxis() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
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
