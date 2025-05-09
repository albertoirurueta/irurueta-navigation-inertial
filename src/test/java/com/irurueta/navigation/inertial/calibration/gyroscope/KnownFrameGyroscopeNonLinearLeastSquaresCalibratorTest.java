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
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class KnownFrameGyroscopeNonLinearLeastSquaresCalibratorTest implements 
        KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener {

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
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(this);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor4() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, this);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor5() throws WrongSizeException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor6() throws WrongSizeException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                this);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor7() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true, 
                this);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor9() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(initialBiasX, initialBiasY,
                initialBiasZ);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor10() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor11() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor12() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor13() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor14() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor15() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor16() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor17() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bgx, bgy, bgz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor18() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bgx, bgy, bgz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor19() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bgx, bgy, bgz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor20() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bgx, bgy, bgz,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor21() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                bgx, bgy, bgz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor22() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                bgx, bgy, bgz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor23() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                true, bgx, bgy, bgz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor24() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bgx, bgy, bgz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor25() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor26() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor27() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor28() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor29() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor30() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor31() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor32() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bgx, bgy, bgz,
                initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor33() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bgx, bgy, bgz,
                initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor34() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bgx, bgy, bgz,
                initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor35() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bgx, bgy, bgz,
                initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor36() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                bgx, bgy, bgz, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor37() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                bgx, bgy, bgz, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor38() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bgx, bgy, bgz, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor39() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);
        final var initialSy = mg.getElementAt(1, 1);
        final var initialSz = mg.getElementAt(2, 2);

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bgx, bgy, bgz, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, 0.0, 0.0, 0.0, initialSy, 0.0, 0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor40() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor41() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor42() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor43() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor44() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor45() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor46() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor47() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bgx, bgy, bgz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor48() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bgx, bgy, bgz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor49() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bgx, bgy, bgz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor50() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bgx, bgy, bgz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor51() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                bgx, bgy, bgz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor52() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true,
                bgx, bgy, bgz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(),  0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor53() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bgx, bgy, bgz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor54() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bgx, bgy, bgz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
    void testConstructor55() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var initialBias = bg.getBuffer();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                new double[1]));
    }

    @Test
    void testConstructor56() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var initialBias = bg.getBuffer();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(initialBias, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                new double[1], this));
    }

    @Test
    void testConstructor57() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var initialBias = bg.getBuffer();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, new double[1]));
    }

    @Test
    void testConstructor58() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var initialBias = bg.getBuffer();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, initialBias,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, new double[1], this));
    }

    @Test
    void testConstructor59() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var initialBias = bg.getBuffer();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true, initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, new double[1]));
    }

    @Test
    void testConstructor60() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var initialBias = bg.getBuffer();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true, initialBias,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, new double[1], this));
    }

    @Test
    void testConstructor61() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var initialBias = bg.getBuffer();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, new double[1]));
    }

    @Test
    void testConstructor62() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var initialBias = bg.getBuffer();
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                initialBias, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, new double[1]));
    }

    @Test
    void testConstructor63() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m2));
    }

    @Test
    void testConstructor64() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m1,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m2,
                this));
    }

    @Test
    void testConstructor65() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m2));
    }

    @Test
    void testConstructor66() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m2, this));
    }

    @Test
    void testConstructor67() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m2));
    }

    @Test
    void testConstructor68() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m2, this));
    }

    @Test
    void testConstructor69() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m2));
    }

    @Test
    void testConstructor70() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bg, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m2, this));
    }

    @Test
    void testConstructor71() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m1,
                mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m2,
                mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                m4));
    }

    @Test
    void testConstructor72() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m1,
                mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m2,
                mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                m4));
    }

    @Test
    void testConstructor73() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, m4));
    }

    @Test
    void testConstructor74() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m1, mg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m2, mg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, m4, this));
    }

    @Test
    void testConstructor75() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, mg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, m4));
    }

    @Test
    void testConstructor76() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, mg,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m1, mg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m2, mg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, m4, this));
    }

    @Test
    void testConstructor77() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bg, mg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m1, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m2, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, m4));
    }

    @Test
    void testConstructor78() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, true,
                bg, mg, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m1, mg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m2, mg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, m4, this));
    }

    @Test
    void testConstructor79() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, gg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m1,
                mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m2,
                mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                mg, m6));
    }

    @Test
    void testConstructor80() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg, mg, gg, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(m1,
                mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                mg, m6, this));
    }

    @Test
    void testConstructor81() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, gg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, mg, m6));
    }

    @Test
    void testConstructor82() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, bg, mg, gg, 
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(KnownFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS, 
                calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, bg, mg, m6, this));
    }

    @Test
    void testConstructor83() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, mg, gg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, mg, m6));
    }

    @Test
    void testConstructor84() throws WrongSizeException {
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true, bg, mg, gg, 
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                true, bg, mg, m6, this));
    }

    @Test
    void testConstructor85() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bg, mg, gg);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy,calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, mg, m6));
    }

    @Test
    void testConstructor86() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements, 
                true, bg, mg, gg, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        var angularSpeed2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
        final var bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final var biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0, 2, 2,
                new double[]{initialSx, initialMyx, initialMzx, initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final var gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                measurements, true, bg, mg, m6, this));
    }

    @Test
    void testGetSetInitialBiasX() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);

        // set new value
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);

        calibrator.setInitialBiasX(initialBiasX);

        // check
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
    }

    @Test
    void testGetSetInitialBiasY() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);

        // set new value
        final var bg = generateBg();
        final var initialBiasY = bg.getElementAtIndex(1);

        calibrator.setInitialBiasY(initialBiasY);

        // check
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
    }

    @Test
    void testGetSetInitialBiasZ() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new value
        final var bg = generateBg();
        final var initialBiasZ = bg.getElementAtIndex(2);

        calibrator.setInitialBiasZ(initialBiasZ);

        // check
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testGetSetInitialBiasAngularSpeedX() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasX1 = calibrator.getInitialBiasAngularSpeedX();

        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasX1.getUnit());

        // set new value
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);

        final var biasX2 = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasX(biasX2);

        // check
        final var biasX3 = calibrator.getInitialBiasAngularSpeedX();
        final var biasX4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(biasX4);

        assertEquals(biasX2, biasX3);
        assertEquals(biasX2, biasX4);
    }

    @Test
    void testGetSetInitialBiasAngularSpeedY() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasY1 = calibrator.getInitialBiasAngularSpeedY();

        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasY1.getUnit());

        // set new value
        final var bg = generateBg();
        final var initialBiasY = bg.getElementAtIndex(1);

        final var biasY2 = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasY(biasY2);

        // check
        final var biasY3 = calibrator.getInitialBiasAngularSpeedY();
        final var biasY4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(biasY4);

        assertEquals(biasY2, biasY3);
        assertEquals(biasY2, biasY4);
    }

    @Test
    void testGetSetInitialBiasAngularSpeedZ() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasZ1 = calibrator.getInitialBiasAngularSpeedZ();

        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasZ1.getUnit());

        // set new value
        final var bg = generateBg();
        final var initialBiasZ = bg.getElementAtIndex(2);

        final var biasZ2 = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasZ(biasZ2);

        // check
        final var biasZ3 = calibrator.getInitialBiasAngularSpeedZ();
        final var biasZ4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(biasZ4);

        assertEquals(biasZ2, biasZ3);
        assertEquals(biasZ2, biasZ4);
    }

    @Test
    void testSetInitialBiasCoordinates1() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new values
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);

        calibrator.setInitialBias(initialBiasX, initialBiasY, initialBiasZ);

        // check
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testSetInitialBiasCoordinates2() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new values
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);

        final var bgx = new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy = new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz = new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBias(bgx, bgy, bgz);

        // check
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testGetSetInitialBiasAsTriad() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        final var triad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final var triad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        // set new values
        final var bg = generateBg();
        final var initialBiasX = bg.getElementAtIndex(0);
        final var initialBiasY = bg.getElementAtIndex(1);
        final var initialBiasZ = bg.getElementAtIndex(2);

        final var triad3 = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, 
                initialBiasX, initialBiasY, initialBiasZ);
        calibrator.setInitialBias(triad3);

        final var triad4 = calibrator.getInitialBiasAsTriad();
        final var triad5 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(triad5);

        assertEquals(triad3, triad4);
        assertEquals(triad3, triad5);
    }

    @Test
    void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialSx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(initialSx);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialSy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(initialSy);

        // check
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialSz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(initialSz);

        // check
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialMxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(initialMxy);

        // check
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialMxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(initialMxz);

        // check
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialMyx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(initialMyx);

        // check
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialMyz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(initialMyz);

        // check
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialMzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(initialMzx);

        // check
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final var mg = generateMg();
        final var initialMzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(initialMzy);

        // check
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

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

        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

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
    void testGetSetInitialBiasArray() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertArrayEquals(new double[3], calibrator.getInitialBias(), 0.0);

        // set new value
        final var bg = generateBg();
        final var initialBias1 = bg.getBuffer();

        calibrator.setInitialBias(initialBias1);

        // check
        final var initialBias2 = calibrator.getInitialBias();
        final var initialBias3 = new double[3];
        calibrator.getInitialBias(initialBias3);

        assertArrayEquals(initialBias1, initialBias2, 0.0);
        assertArrayEquals(initialBias1, initialBias3, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(new double[1]));
    }

    @Test
    void testGetSetInitialBiasAsMatrix() throws LockedException, WrongSizeException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getInitialBiasAsMatrix());

        // set new value
        final var bg = generateBg();
        calibrator.setInitialBias(bg);

        // check
        final var initialBias1 = calibrator.getInitialBiasAsMatrix();
        final var initialBias2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(initialBias2);

        assertEquals(bg, initialBias1);
        assertEquals(bg, initialBias2);

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
    void testGetSetInitialMg() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        final var initialMg1 = calibrator.getInitialMg();

        assertEquals(new Matrix(3, 3), initialMg1);

        // set new value
        final var mg = generateMg();

        calibrator.setInitialMg(mg);

        // check
        final var initialMg2 = calibrator.getInitialMg();
        final var initialMg3 = new Matrix(3, 3);
        calibrator.getInitialMg(initialMg3);

        assertEquals(mg, initialMg2);
        assertEquals(mg, initialMg3);

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
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check
        assertEquals(new Matrix(3, 3), calibrator.getInitialGg());

        // set new value
        final var gg = generateGg();

        calibrator.setInitialGg(gg);

        // check
        final var initialGg1 = calibrator.getInitialGg();
        final var initialGg2 = new Matrix(3, 3);
        calibrator.getInitialGg(initialGg2);

        assertEquals(gg, initialGg1);
        assertEquals(gg, initialGg2);

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
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testIsReady() throws LockedException {
        final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isReady());

        // set enough measurements
        final var measurements1 = Arrays.asList(new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics());

        calibrator.setMeasurements(measurements1);

        assertTrue(calibrator.isReady());

        // set too few measurements
        final var measurements2 = Arrays.asList(new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics());

        calibrator.setMeasurements(measurements2);

        assertFalse(calibrator.isReady());
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
            for (var i = 0; i < KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
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
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                    false, bg, mg, gg, this);

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

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

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

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
                final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
                final var longitude = Math.toRadians(
                        randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
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

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                    false, bg, mg, gg, this);

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

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertNotNull(estimatedGg);

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
            for (var i = 0; i < KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
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
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueKinematics, errors, random);

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

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
    void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise() throws WrongSizeException,
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

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
            final var random = new Random();
            for (var i = 0; i < KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
                final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
                final var longitude = Math.toRadians(
                        randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
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

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final var calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertNotNull(estimatedGg);

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() < 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateEnd++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
    }

    private void checkLocked(final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
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
        assertThrows(LockedException.class, () -> calibrator.setInitialScalingFactors(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((Matrix) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialMg(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialGg(null));
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix bg, final Matrix mg, final Matrix gg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) throws WrongSizeException {

        final var estimatedBiases = calibrator.getEstimatedBiases();
        assertArrayEquals(bg.getBuffer(), estimatedBiases, 0.0);

        final var estimatedBiases2 = new double[3];
        calibrator.getEstimatedBiases(estimatedBiases2);
        assertArrayEquals(estimatedBiases, estimatedBiases2, 0.0);

        final var bg2 = new Matrix(3, 1);
        calibrator.getEstimatedBiasesAsMatrix(bg2);

        assertEquals(bg, bg2);

        assertEquals(bg.getElementAtIndex(0), calibrator.getEstimatedBiasX(), 0.0);
        assertEquals(bg.getElementAtIndex(1), calibrator.getEstimatedBiasY(), 0.0);
        assertEquals(bg.getElementAtIndex(2), calibrator.getEstimatedBiasZ(), 0.0);

        final var bgx1 = calibrator.getEstimatedBiasAngularSpeedX();
        final var bgx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        assertEquals(bgx1.getValue().doubleValue(), calibrator.getEstimatedBiasX(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        final var bgy1 = calibrator.getEstimatedBiasAngularSpeedY();
        final var bgy2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        assertEquals(bgy1.getValue().doubleValue(), calibrator.getEstimatedBiasY(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        final var bgz1 = calibrator.getEstimatedBiasAngularSpeedZ();
        final var bgz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
        assertEquals(bgz1.getValue().doubleValue(), calibrator.getEstimatedBiasZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

        final var bTriad1 = calibrator.getEstimatedBiasAsTriad();
        assertEquals(bTriad1.getValueX(), calibrator.getEstimatedBiasX(), 0.0);
        assertEquals(bTriad1.getValueY(), calibrator.getEstimatedBiasY(), 0.0);
        assertEquals(bTriad1.getValueZ(), calibrator.getEstimatedBiasZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bTriad1.getUnit());
        final var bTriad2 = new AngularSpeedTriad();
        calibrator.getEstimatedBiasAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);

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

    private static void assertCovariance(final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        assertNotNull(calibrator.getEstimatedBiasXVariance());

        assertNotNull(calibrator.getEstimatedBiasXVariance());
        assertNotNull(calibrator.getEstimatedBiasXStandardDeviation());
        final var stdBx1 = calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed();
        assertNotNull(stdBx1);
        final var stdBx2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(stdBx2));
        assertEquals(stdBx1, stdBx2);

        assertNotNull(calibrator.getEstimatedBiasYVariance());
        assertNotNull(calibrator.getEstimatedBiasYStandardDeviation());
        final var stdBy1 = calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed();
        assertNotNull(stdBy1);
        final var stdBy2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(stdBy2));
        assertEquals(stdBy1, stdBy2);

        assertNotNull(calibrator.getEstimatedBiasZVariance());
        assertNotNull(calibrator.getEstimatedBiasZStandardDeviation());
        final var stdBz1 = calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed();
        assertNotNull(stdBz1);
        final var stdBz2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(stdBz2));
        assertEquals(stdBz1, stdBz2);

        final var std1 = calibrator.getEstimatedBiasStandardDeviation();
        assertEquals(std1.getValueX(), calibrator.getEstimatedBiasXStandardDeviation(), 0.0);
        assertEquals(std1.getValueY(), calibrator.getEstimatedBiasYStandardDeviation(), 0.0);
        assertEquals(std1.getValueZ(), calibrator.getEstimatedBiasZStandardDeviation(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final var std2 = new AngularSpeedTriad();
        calibrator.getEstimatedBiasStandardDeviation(std2);

        final var avgStd = (calibrator.getEstimatedBiasXStandardDeviation()
                + calibrator.getEstimatedBiasYStandardDeviation()
                + calibrator.getEstimatedBiasZStandardDeviation()) / 3.0;
        assertEquals(avgStd, calibrator.getEstimatedBiasStandardDeviationAverage(), 0.0);
        final var avg1 = calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed();
        assertEquals(avgStd, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avg1.getUnit());
        final var avg2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(avg2);
        assertEquals(avg1, avg2);

        assertEquals(std1.getNorm(), calibrator.getEstimatedBiasStandardDeviationNorm(), ABSOLUTE_ERROR);
        final var norm1 = calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed();
        assertEquals(std1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        final var norm2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(norm2);
        assertEquals(norm1, norm2);
    }

    private static void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (var j = 0; j < 18; j++) {
            final var colIsZero = j == 8 || j == 10 || j == 11;
            for (var i = 0; i < 18; i++) {
                final var rowIsZero = i == 8 || i == 10 || i == 11;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (var i = 0; i < 21; i++) {
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
