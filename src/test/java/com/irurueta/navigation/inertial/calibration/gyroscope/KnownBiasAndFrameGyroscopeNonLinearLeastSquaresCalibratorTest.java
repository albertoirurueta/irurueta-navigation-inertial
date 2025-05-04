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
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorTest implements
        KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener {

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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), b1);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), b1);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), b1);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), b1);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), b1);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), b1);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                true);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), b1);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), b1);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                biasX, biasY, biasZ);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                biasX, biasY, biasZ, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                true, biasX, biasY, biasZ);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                true, biasX, biasY, biasZ, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bx, by, bz);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bx, by, bz, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bx, by, bz);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bx, by, bz,
                this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                bx, by, bz);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                bx, by, bz, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor23() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                true, bx, by, bz);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor24() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                true, bx, by, bz, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor25() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor26() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor27() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor28() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor29() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor30() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                true, biasX, biasY, biasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor31() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                true, biasX, biasY, biasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor32() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bx, by, bz, initialSx,
                initialSy, initialSz);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor33() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bx, by, bz, 
                initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor34() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bx, by, bz, 
                initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor35() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bx, by, bz,
                initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor36() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                bx, by, bz, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor37() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                bx, by, bz, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasX, biasTriad1.getValueX(), 0.0);
        assertEquals(biasY, biasTriad1.getValueY(), 0.0);
        assertEquals(biasZ, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor38() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bx, by, bz, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor39() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bx, by, bz, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor40() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor41() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor42() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor43() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor44() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor45() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor46() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor47() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bx, by, bz, 
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor48() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bx, by, bz, 
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, 
                this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor49() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bx, by, bz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor50() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bx, by, bz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy,
                this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor51() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                bx, by, bz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor52() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                bx, by, bz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor53() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bx, by, bz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor54() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bx, by, bz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg3 = new Matrix(3, 3);
        calibrator.getInitialMg(mg3);
        assertEquals(mg1, mg3);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor55() throws WrongSizeException {
        final var bg = generateBg();
        final var bias = bg.getBuffer();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bias);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(new double[1]));
    }

    @Test
    void testConstructor56() throws WrongSizeException {
        final var bg = generateBg();
        final var bias = bg.getBuffer();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bias, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(new double[1], this));
    }

    @Test
    void testConstructor57() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var bias = bg.getBuffer();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bias);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, new double[1]));
    }

    @Test
    void testConstructor58() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var bias = bg.getBuffer();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bias, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, new double[1],
                        this));
    }

    @Test
    void testConstructor59() throws WrongSizeException {
        final var bg = generateBg();
        final var bias = bg.getBuffer();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                bias);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                        new double[1]));
    }

    @Test
    void testConstructor60() throws WrongSizeException {
        final var bg = generateBg();
        final var bias = bg.getBuffer();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bias,
                this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, new double[1],
                        this));
    }

    @Test
    void testConstructor61() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var bias = bg.getBuffer();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bias);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true, 
                        new double[1]));
    }

    @Test
    void testConstructor62() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var bias = bg.getBuffer();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bias, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        new double[1], this));
    }

    @Test
    void testConstructor63() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, 
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m2));
    }

    @Test
    void testConstructor64() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m2, this));
    }

    @Test
    void testConstructor65() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m2));
    }

    @Test
    void testConstructor66() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, 
                this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m2, this));
    }

    @Test
    void testConstructor67() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m2));
    }

    @Test
    void testConstructor68() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, 
                this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m1,
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m2, 
                        this));
    }

    @Test
    void testConstructor69() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bg);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        m2));
    }

    @Test
    void testConstructor70() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bg, this);

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
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        m2, this));
    }

    @Test
    void testConstructor71() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, m4));
    }

    @Test
    void testConstructor72() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m1, mg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m2, mg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, m4, this));
    }

    @Test
    void testConstructor73() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m4));
    }

    @Test
    void testConstructor74() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, 
                this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m1, mg, 
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m2, mg,
                        this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m3,
                        this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m4,
                        this));
    }

    @Test
    void testConstructor75() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, 
                mg);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, m4));
    }

    @Test
    void testConstructor76() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, 
                mg, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, m4));
    }

    @Test
    void testConstructor77() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bg, mg);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m4));
    }

    @Test
    void testConstructor78() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bg, mg, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m4));
    }

    @Test
    void testConstructor79() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);
        final var gg = generateGg();

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, gg);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, m6));
    }

    @Test
    void testConstructor80() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);
        final var gg = generateGg();

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, m6, this));
    }

    @Test
    void testConstructor81() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);
        final var gg = generateGg();

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, gg);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, m6));
    }

    @Test
    void testConstructor82() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);
        final var gg = generateGg();

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, gg, 
                this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m1, mg, gg,
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, m2, mg, gg,
                        this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m3, gg,
                        this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, m4, gg,
                        this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, m5,
                        this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, m6,
                        this));
    }

    @Test
    void testConstructor83() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);
        final var gg = generateGg();

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                bg, mg, gg);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, mg, m6));
    }

    @Test
    void testConstructor84() throws WrongSizeException {
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);
        final var gg = generateGg();

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, 
                bg, mg, gg, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m1, mg, gg,
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, m2, mg, gg,
                        this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, m3, gg,
                        this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, m4, gg,
                        this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, mg, m5,
                        this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, mg, m6,
                        this));
    }

    @Test
    void testConstructor85() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);
        final var gg = generateGg();

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bg, mg, gg);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        bg, mg, m6));
    }

    @Test
    void testConstructor86() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);
        final var gg = generateGg();

        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bg, mg, gg, this);

        // check default values
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
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
        final var bz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        assertEquals(bz1, bz2);
        final var bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var b1 = calibrator.getBiasAsMatrix();
        assertEquals(b1, bg);
        final var b2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(b2);
        assertEquals(b1, b2);
        final var biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final var biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                        bg, mg, m6));
    }

    @Test
    void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(initialSx);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialSy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(initialSy);

        // check
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialSz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(initialSz);

        // check
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialMxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(initialMxy);

        // check
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialMxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(initialMxz);

        // check
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialMyx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(initialMyx);

        // check
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialMyz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(initialMyz);

        // check
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialMzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(initialMzx);

        // check
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new value
        final var mg = generateMg();
        final var initialMzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(initialMzy);

        // check
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testSetInitialCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        calibrator.setInitialCrossCouplingErrors(initialMxy, initialMxz, initialMyx, 
                initialMyz, initialMzx, initialMzy);

        // check
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testSetInitialScalingFactorsAndCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);
        final var initialMxy = mg.getElementAt(0, 1);
        final var initialMxz = mg.getElementAt(0, 2);
        final var initialMyx = mg.getElementAt(1, 0);
        final var initialMyz = mg.getElementAt(1, 2);
        final var initialMzx = mg.getElementAt(2, 0);
        final var initialMzy = mg.getElementAt(2, 1);

        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

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
    void testGetSetInitialMg() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final var mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);

        // set new value
        final var mg2 = generateMg();
        calibrator.setInitialMg(mg2);

        // check
        final var mg3 = calibrator.getInitialMg();
        final var mg4 = new Matrix(3, 3);
        calibrator.getInitialMg(mg4);

        assertEquals(mg2, mg3);
        assertEquals(mg2, mg4);

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
    void testGetSetInitialGgg() throws WrongSizeException, LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final var gg1 = calibrator.getInitialGg();
        assertEquals(new Matrix(3, 3), gg1);

        // set new value
        final var gg2 = generateGg();
        calibrator.setInitialGg(gg2);

        // check
        final var gg3 = calibrator.getInitialGg();
        final var gg4 = new Matrix(3, 3);
        calibrator.getInitialGg(gg4);

        assertEquals(gg2, gg3);
        assertEquals(gg2, gg4);

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
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetSetBiasX() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new values
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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new values
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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final var bias1 = calibrator.getBias();

        assertArrayEquals(new double[3], bias1, 0.0);

        // set new values
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
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final var bias1 = calibrator.getBiasAsMatrix();

        assertEquals(new Matrix(3, 1), bias1);

        // set new values
        final var bias2 = generateBg();
        calibrator.setBias(bias2);

        // check
        final var bias3 = calibrator.getBiasAsMatrix();
        final var bias4 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bias4);

        assertEquals(bias2, bias3);
        assertEquals(bias2, bias4);
    }

    @Test
    void testGetSetBiasAsTriad() throws LockedException {
        final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueKinematics, errors, random);

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation,
                        angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, false,
                    bg, mg, gg, this);

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

            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() < 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
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
            final var random = new Random();
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

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation,
                        angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                    false, this);

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

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException {

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
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation,
                        angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                    true, bg, mg, gg, this);

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

            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() < 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
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
            final var random = new Random();
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

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation,
                        angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                    true, this);

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

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateEnd++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
    }

    private static void checkLocked(final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
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
        assertThrows(LockedException.class, () -> calibrator.setInitialCrossCouplingErrors(0.0, 0.0,
                0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMg(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialGg(null));
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
            final Matrix mg, final Matrix gg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {

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

    private static void checkCommonAxisCovariance(final Matrix covariance) {
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

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(18, covariance.getRows());
        assertEquals(18, covariance.getColumns());

        for (var i = 0; i < 18; i++) {
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
