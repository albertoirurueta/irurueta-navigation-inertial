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
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorTest implements
        KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener {

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

    private int calibrateStart;
    private int calibrateEnd;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
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
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
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
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
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
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
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
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
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
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(true, 
                this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
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
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements, 
                true);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
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
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                true, this);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
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
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(biasX, biasY, biasZ);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(biasY, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(biasZ, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(biasX, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, this);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                true, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, true);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, true, this);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor17() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(bx, by, bz);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
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
    }

    @Test
    void testConstructor18() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(bx, by, bz, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements, bx, by, bz);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements, bx, by, bz,
                this);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor21() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(bx, by, bz,
                true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor22() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(bx, by, bz,
                true, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor23() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements, bx, by, bz,
                true);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testConstructor24() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements, bx, by, bz,
                true, this);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final var bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    }

    @Test
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final var measurements = Collections.<FrameBodyKinematics>emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetSetBiasX() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasX(), 0.0);

        // set new value
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);

        calibrator.setBiasX(biasX);

        // check
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
    }

    @Test
    void testGetSetBiasY() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasY(), 0.0);

        // set new value
        final var bg = generateBg();
        final var biasY = bg.getElementAtIndex(1);

        calibrator.setBiasY(biasY);

        // check
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
    }

    @Test
    void testGetSetBiasZ() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new value
        final var bg = generateBg();
        final var biasZ = bg.getElementAtIndex(2);

        calibrator.setBiasZ(biasZ);

        // check
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
    }

    @Test
    void testGetSetBiasAngularSpeedX() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final var bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());

        // set new value
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var bx2 = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasX(bx2);

        // check
        final var bx3 = calibrator.getBiasAngularSpeedX();
        final var bx4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    void testGetSetBiasAngularSpeedY() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final var by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());

        // set new value
        final var bg = generateBg();
        final var biasY = bg.getElementAtIndex(1);
        final var by2 = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasY(by2);

        // check
        final var by3 = calibrator.getBiasAngularSpeedY();
        final var by4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    void testGetSetBiasAngularSpeedZ() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final var bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());

        // set new value
        final var bg = generateBg();
        final var biasZ = bg.getElementAtIndex(2);
        final var bz2 = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasZ(bz2);

        // check
        final var bz3 = calibrator.getBiasAngularSpeedZ();
        final var bz4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    void testSetBiasCoordinates1() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new value
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);

        calibrator.setBiasCoordinates(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
    }

    @Test
    void testSetBiasCoordinates2() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new value
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);

        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasCoordinates(bx, by, bz);

        // check
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
    }

    @Test
    void testGetSetBias() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);

        // set new value
        final var bg = generateBg();
        final var bias2 = bg.getBuffer();

        calibrator.setBias(bias2);

        // check
        final var bias3 = calibrator.getBias();
        final var bias4 = new double[3];
        calibrator.getBias(bias4);

        assertArrayEquals(bias2, bias3, 0.0);
        assertArrayEquals(bias2, bias4, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setBias(new double[1]));
    }

    @Test
    void testGetSetBiasAsMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final var bg1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), bg1);

        // set new value
        final var bg2 = generateBg();

        calibrator.setBias(bg2);

        // check
        final var bg3 = calibrator.getBiasAsMatrix();
        final var bg4 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg4);

        assertEquals(bg2, bg3);
        assertEquals(bg2, bg4);

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
    void testGetSetBiasAsTriad() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

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
    void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, CalibrationException {

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

        final var random = new Random();
        final var measurements = new ArrayList<FrameBodyKinematics>();
        for (var i = 0; i < KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random);

            final var measurement = new FrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                    TIME_INTERVAL_SECONDS);
            measurements.add(measurement);
        }

        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements, 
                biasX, biasY, biasZ, false, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(0, calibrateStart);
        assertEquals(0, calibrateEnd);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(1, calibrateStart);
        assertEquals(1, calibrateEnd);

        final var estimatedMg = calibrator.getEstimatedMg();
        final var estimatedGg = calibrator.getEstimatedGg();

        assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
        assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMg, estimatedGg, calibrator);
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
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var random = new Random();
            final var measurements = new ArrayList<FrameBodyKinematics>();
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
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                        errors, random);

                final var measurement = new FrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                        TIME_INTERVAL_SECONDS);
                measurements.add(measurement);
            }

            final var biasX = bg.getElementAtIndex(0);
            final var biasY = bg.getElementAtIndex(1);
            final var biasZ = bg.getElementAtIndex(2);

            final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                    biasX, biasY, biasZ, false, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
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

            final var random = new Random();
            final var measurements = new ArrayList<FrameBodyKinematics>();
            for (var i = 0; i < KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

                final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
                final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
                final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
                final var nedPosition = new NEDPosition(latitude, longitude, height);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                        errors, random);

                final var measurement = new FrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                        TIME_INTERVAL_SECONDS);
                measurements.add(measurement);
            }

            final var biasX = bg.getElementAtIndex(0);
            final var biasY = bg.getElementAtIndex(1);
            final var biasZ = bg.getElementAtIndex(2);

            final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                    biasX, biasY, biasZ, false, this);

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

            // check
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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
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

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var random = new Random();
        final var measurements = new ArrayList<FrameBodyKinematics>();
        for (var i = 0; i < KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

            final var measurement = new FrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                    TIME_INTERVAL_SECONDS);
            measurements.add(measurement);
        }

        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);

        final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, true, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(0, calibrateStart);
        assertEquals(0, calibrateEnd);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(1, calibrateStart);
        assertEquals(1, calibrateEnd);

        final var estimatedMg = calibrator.getEstimatedMg();
        final var estimatedGg = calibrator.getEstimatedGg();

        assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
        assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMg, estimatedGg, calibrator);
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

            final var random = new Random();
            final var measurements = new ArrayList<FrameBodyKinematics>();
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
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                        errors, random);

                final var measurement = new FrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                        TIME_INTERVAL_SECONDS);
                measurements.add(measurement);
            }

            final var biasX = bg.getElementAtIndex(0);
            final var biasY = bg.getElementAtIndex(1);
            final var biasZ = bg.getElementAtIndex(2);

            final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                    biasX, biasY, biasZ, true, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
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

            final var random = new Random();
            final var measurements = new ArrayList<FrameBodyKinematics>();
            for (var i = 0; i < KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

                final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
                final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
                final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
                final var nedPosition = new NEDPosition(latitude, longitude, height);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                        errors, random);

                final var measurement = new FrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                        TIME_INTERVAL_SECONDS);
                measurements.add(measurement);
            }

            final var biasX = bg.getElementAtIndex(0);
            final var biasY = bg.getElementAtIndex(1);
            final var biasZ = bg.getElementAtIndex(2);

            final var calibrator = new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                    biasX, biasY, biasZ, true, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            calibrator.calibrate();

            // check
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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateEnd++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
    }

    private static void checkLocked(final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasX(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasY(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasZ(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasCoordinates(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasCoordinates(null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setBias((AngularSpeedTriad) null));
        assertThrows(LockedException.class, () -> calibrator.setBias((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setBias((Matrix) null));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix mg, final Matrix gg, final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator) {

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
