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

class KnownFrameAccelerometerNonLinearLeastSquaresCalibratorTest implements 
        KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener {

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(this);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, this);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
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
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true, 
                this);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, this);

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(0.0, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var biasMatrix1 = new Matrix(3, 1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
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
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor17() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(bax, bay, baz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor18() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(bax, bay, baz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bax, bay, baz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bax, bay, baz,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor21() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true, 
                bax, bay, baz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor22() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true, 
                bax, bay, baz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor23() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, bax, bay, baz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), initialBiasX, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
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
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor24() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, bax, bay, baz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor25() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor26() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), initialBiasX, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor27() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor28() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor29() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true, 
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), initialBiasX, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor30() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor31() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor32() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor33() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(bax, bay, baz, 
                initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor34() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(bax, bay, baz, 
                initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor35() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bax, bay, baz, 
                initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor36() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bax, bay, baz,
                initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor37() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true, 
                bax, bay, baz, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor38() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true, 
                bax, bay, baz, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor39() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, bax, bay, baz, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor40() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var ma = generateMaGeneral();
        final var initialSx = ma.getElementAt(0, 0);
        final var initialSy = ma.getElementAt(1, 1);
        final var initialSz = ma.getElementAt(2, 2);

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                true, bax, bay, baz, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor41() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, 
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor42() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, 
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor43() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor44() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor45() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor46() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor47() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor48() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(bax, bay, baz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor49() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(bax, bay, baz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor50() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bax, bay, baz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor51() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, bax, bay, baz,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor52() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                bax, bay, baz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor53() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                bax, bay, baz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor54() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, bax, bay, baz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor55() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var bax = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, bax, bay, baz, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
    }

    @Test
    void testConstructor56() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = new double[]{initialBiasX, initialBiasY, initialBiasZ};

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                new double[1]));
    }

    @Test
    void testConstructor57() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = new double[]{initialBiasX, initialBiasY, initialBiasZ};

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(initialBias, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                new double[1], this));
    }

    @Test
    void testConstructor58() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = new double[]{initialBiasX, initialBiasY, initialBiasZ};

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, new double[1]));
    }

    @Test
    void testConstructor59() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = new double[]{initialBiasX, initialBiasY, initialBiasZ};

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, initialBias,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
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
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, new double[1]));
    }

    @Test
    void testConstructor60() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = new double[]{initialBiasX, initialBiasY, initialBiasZ};

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                true, new double[1]));
    }

    @Test
    void testConstructor61() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = new double[]{initialBiasX, initialBiasY, initialBiasZ};

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBias, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                true, new double[1], this));
    }

    @Test
    void testConstructor62() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = new double[]{initialBiasX, initialBiasY, initialBiasZ};

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                true, new double[1]));
    }

    @Test
    void testConstructor63() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = new double[]{initialBiasX, initialBiasY, initialBiasZ};

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, initialBias, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                true, new double[1]));
    }

    @Test
    void testConstructor64() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1));
    }

    @Test
    void testConstructor65() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(initialBias, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, this));
    }

    @Test
    void testConstructor66() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, m1));
    }

    @Test
    void testConstructor67() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, initialBias,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, this));
    }

    @Test
    void testConstructor68() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                true, m1));
    }

    @Test
    void testConstructor69() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBias, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                true, m1, this));
    }

    @Test
    void testConstructor70() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, initialBias);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, true, m1));
    }

    @Test
    void testConstructor71() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, initialBias, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                measurements, true, m1, this));
    }

    @Test
    void testConstructor72() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(initialBias, ma);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBias, m1));
    }

    @Test
    void testConstructor73() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var initialBias = Matrix.newFromArray(new double[]{
                initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(initialBias, ma,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBias, m1));
    }

    @Test
    void testConstructor74() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, initialBias,
                ma);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBias, m1));
    }

    @Test
    void testConstructor75() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements, initialBias, ma,
                this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBias, m1));
    }

    @Test
    void testConstructor76() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBias, ma);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBias, m1));
    }

    @Test
    void testConstructor77() throws WrongSizeException {
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(true,
                initialBias, ma, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBias, m1));
    }

    @Test
    void testConstructor78() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, initialBias, ma);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBias, m1));
    }

    @Test
    void testConstructor79() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);
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

        final var initialBias = Matrix.newFromArray(new double[]{initialBiasX, initialBiasY, initialBiasZ});

        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, initialBias, ma, this);

        // check default values
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(initialBiasX, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(initialBiasY, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(initialBiasZ, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasX, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(initialBiasY, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(initialBiasZ, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
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
        final var bias1 = new double[]{initialBiasX, initialBiasY, initialBiasZ};
        assertArrayEquals(bias1, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final var biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(biasMatrix1, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
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
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                m1, ma));
        assertThrows(IllegalArgumentException.class, () -> new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
                initialBias, m1));
    }

    @Test
    void testGetSetInitialBiasX() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);

        // set new value
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);

        calibrator.setInitialBiasX(initialBiasX);

        // check
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
    }

    @Test
    void testGetSetInitialBiasY() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);

        // set new value
        final var ba = generateBa();
        final var initialBiasY = ba.getElementAtIndex(1);

        calibrator.setInitialBiasY(initialBiasY);

        // check
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
    }

    @Test
    void testGetSetInitialBiasZ() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new value
        final var ba = generateBa();
        final var initialBiasZ = ba.getElementAtIndex(2);

        calibrator.setInitialBiasZ(initialBiasZ);

        // check
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testGetSetInitialBiasXAsAcceleration() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasX1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasX1.getUnit());

        // set new value
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);

        final var biasX2 = new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setInitialBiasX(biasX2);

        // check
        final var biasX3 = calibrator.getInitialBiasXAsAcceleration();
        final var biasX4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(biasX4);

        assertEquals(biasX2, biasX3);
        assertEquals(biasX2, biasX4);
    }

    @Test
    void testGetSetInitialBiasYAsAcceleration() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasY1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasY1.getUnit());

        // set new value
        final var ba = generateBa();
        final var initialBiasY = ba.getElementAtIndex(1);

        final var biasY2 = new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setInitialBiasY(biasY2);

        // check
        final var biasY3 = calibrator.getInitialBiasYAsAcceleration();
        final var biasY4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(biasY4);

        assertEquals(biasY2, biasY3);
        assertEquals(biasY2, biasY4);
    }

    @Test
    void testGetSetInitialBiasZAsAcceleration() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var biasZ1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasZ1.getUnit());

        // set new value
        final var ba = generateBa();
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var biasZ2 = new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setInitialBiasZ(biasZ2);

        // check
        final var biasZ3 = calibrator.getInitialBiasZAsAcceleration();
        final var biasZ4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(biasZ4);

        assertEquals(biasZ2, biasZ3);
        assertEquals(biasZ2, biasZ4);
    }

    @Test
    void testSetInitialBias1() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new values
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        calibrator.setInitialBias(initialBiasX, initialBiasY, initialBiasZ);

        // check
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testSetInitialBias2() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        final var def = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // check default values
        assertEquals(def, calibrator.getInitialBiasXAsAcceleration());
        assertEquals(def, calibrator.getInitialBiasYAsAcceleration());
        assertEquals(def, calibrator.getInitialBiasZAsAcceleration());

        // set new values
        final var ba = generateBa();
        final var initialBiasX = new Acceleration(ba.getElementAtIndex(0), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBiasY = new Acceleration(ba.getElementAtIndex(1), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBiasZ = new Acceleration(ba.getElementAtIndex(2), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setInitialBias(initialBiasX, initialBiasY, initialBiasZ);

        // check
        assertEquals(initialBiasX, calibrator.getInitialBiasXAsAcceleration());
        assertEquals(initialBiasY, calibrator.getInitialBiasYAsAcceleration());
        assertEquals(initialBiasZ, calibrator.getInitialBiasZAsAcceleration());
    }

    @Test
    void testGetSetInitialBiasAsTriad() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        final var triad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        // set new values
        final var ba = generateBa();
        final var initialBiasX = ba.getElementAtIndex(0);
        final var initialBiasY = ba.getElementAtIndex(1);
        final var initialBiasZ = ba.getElementAtIndex(2);

        final var triad3 = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                initialBiasX, initialBiasY, initialBiasZ);
        calibrator.setInitialBias(triad3);

        final var triad4 = calibrator.getInitialBiasAsTriad();
        final var triad5 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(triad5);

        assertEquals(triad3, triad4);
        assertEquals(triad3, triad5);
    }

    @Test
    void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new value
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
    void testSetInitialCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final var ma = generateMaGeneral();
        final var initialMxy = ma.getElementAt(0, 1);
        final var initialMxz = ma.getElementAt(0, 2);
        final var initialMyx = ma.getElementAt(1, 0);
        final var initialMyz = ma.getElementAt(1, 2);
        final var initialMzx = ma.getElementAt(2, 0);
        final var initialMzy = ma.getElementAt(2, 1);

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
    void testGetSetInitialBiasAsArray() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var initialBias1 = calibrator.getInitialBias();
        final var initialBias2 = new double[3];
        calibrator.getInitialBias(initialBias2);

        assertArrayEquals(new double[3], initialBias1, 0.0);
        assertArrayEquals(initialBias1, initialBias2, 0.0);

        // set new value
        final var ba = generateBa();
        final var initialBias3 = ba.getBuffer();

        calibrator.setInitialBias(initialBias3);

        // check
        final var initialBias4 = calibrator.getInitialBias();
        final var initialBias5 = new double[3];
        calibrator.getInitialBias(initialBias5);

        assertArrayEquals(initialBias3, initialBias4, 0.0);
        assertArrayEquals(initialBias3, initialBias5, 0.0);

        // Force Illegal ArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(new double[1]));
    }

    @Test
    void testGetSetInitialBiasAsMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var def = new Matrix(3, 1);

        final var initialBias1 = calibrator.getInitialBiasAsMatrix();
        final var initialBias2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(initialBias2);

        assertEquals(def, initialBias1);
        assertEquals(def, initialBias2);

        // set new value
        final var ba = generateBa();

        calibrator.setInitialBias(ba);

        // check
        final var initialBias3 = calibrator.getInitialBiasAsMatrix();
        final var initialBias4 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(initialBias4);

        assertEquals(ba, initialBias3);
        assertEquals(ba, initialBias4);

        // Force IllegalArgumentException
        final var wrong1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBiasAsMatrix(wrong1));
        final var wrong2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBiasAsMatrix(wrong2));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(wrong1));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(wrong2));
    }

    @Test
    void testGetSetInitialMa() throws WrongSizeException, LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final var def = new Matrix(3, 3);
        assertEquals(def, calibrator.getInitialMa());

        // set new value
        final var ma1 = generateMaGeneral();
        calibrator.setInitialMa(ma1);

        // check
        final var ma2 = calibrator.getInitialMa();
        final var ma3 = new Matrix(3, 3);
        calibrator.getInitialMa(ma3);

        assertEquals(ma1, ma2);
        assertEquals(ma1, ma3);

        // Force IllegalArgumentException
        final var wrong1 = new Matrix(1, 1);
        final var wrong2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialMa(wrong1));
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialMa(wrong2));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialMa(wrong1));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialMa(wrong2));
    }

    @Test
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        calibrator.setMeasurements(measurements);

        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());
        assertEquals(KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
                calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testIsReady() throws LockedException {
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isReady());

        // set enough measurements
        final var measurements1 = Arrays.asList(new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(), new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics());

        calibrator.setMeasurements(measurements1);

        assertTrue(calibrator.isReady());

        // set too few measurements
        final var measurements2 = Arrays.asList(new StandardDeviationFrameBodyKinematics(), 
                new StandardDeviationFrameBodyKinematics(), new StandardDeviationFrameBodyKinematics());

        calibrator.setMeasurements(measurements2);

        assertFalse(calibrator.isReady());
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
        for (var i = 0; i < KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                false, ba, ma, this);

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

        final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
        final var estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
        assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedBa, estimatedMa, calibrator);

        assertNotNull(calibrator.getEstimatedCovariance());
        checkGeneralCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedChiSq() < 0.0);
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
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var random = new Random();
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                        errors, random);

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation,
                        angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have a large number of measurements, there is no need to provide
            // an initial value because the default initial value (all zeros) will
            // typically converge close to the true solution
            final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator);

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
        for (var i = 0; i < KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                false, ba, ma, this);

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

        final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
        final var estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
        assertTrue(ma.equals(estimatedMa, VERY_LARGE_ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedBa, estimatedMa, calibrator);

        assertNotNull(calibrator.getEstimatedCovariance());
        checkGeneralCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedChiSq() < 0.0);
        assertTrue(calibrator.getEstimatedMse() > 0.0);
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

        final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final var measurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
        for (var i = 0; i < KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random);

            final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame, ecefFrame,
                    TIME_INTERVAL_SECONDS, specificForceStandardDeviation, angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, ba, ma, this);

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

        final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
        final var estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
        assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedBa, estimatedMa, calibrator);

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
                final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final var random = new Random();
                final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                        errors, random);

                final var measurement = new StandardDeviationFrameBodyKinematics(measuredKinematics, ecefFrame,
                        ecefFrame, TIME_INTERVAL_SECONDS, specificForceStandardDeviation,
                        angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have a large number of measurements, there is no need to provide
            // an initial value because the default initial value (all zeros) will
            // typically converge close to the true solution
            final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator);

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
        for (var i = 0; i < KnownFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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
        final var calibrator = new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                true, ba, ma, this);

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

        final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
        final var estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
        assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedBa, estimatedMa, calibrator);

        assertNotNull(calibrator.getEstimatedCovariance());
        checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedChiSq() < 0.0);
        assertTrue(calibrator.getEstimatedMse() > 0.0);
    }

    @Override
    public void onCalibrateStart(final KnownFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateEnd++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
    }

    private void checkLocked(final KnownFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasX(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasY(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBiasZ(null));
        assertThrows(LockedException.class,
                () -> calibrator.setInitialBias(0.0, 0.0, 0.0));
        assertThrows(LockedException.class,
                () -> calibrator.setInitialBias(null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((AccelerationTriad) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialSx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialSy(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialSz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMxy(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMxz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMyx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMyz(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMzx(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialMzy(0.0));
        assertThrows(LockedException.class,
                () -> calibrator.setInitialScalingFactors(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((Matrix) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialMa(null));
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix ba, final Matrix ma, final KnownFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator)
            throws WrongSizeException {

        final var estimatedBiases = calibrator.getEstimatedBiases();
        assertArrayEquals(ba.getBuffer(), estimatedBiases, 0.0);

        final var estimatedBiases2 = new double[3];
        calibrator.getEstimatedBiases(estimatedBiases2);
        assertArrayEquals(estimatedBiases, estimatedBiases2, 0.0);

        final var ba2 = new Matrix(3, 1);
        calibrator.getEstimatedBiasesAsMatrix(ba2);

        assertEquals(ba, ba2);

        assertEquals(ba.getElementAtIndex(0), calibrator.getEstimatedBiasFx(), 0.0);
        assertEquals(ba.getElementAtIndex(1), calibrator.getEstimatedBiasFy(), 0.0);
        assertEquals(ba.getElementAtIndex(2), calibrator.getEstimatedBiasFz(), 0.0);

        final var bax1 = calibrator.getEstimatedBiasFxAsAcceleration();
        final var bax2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasFxAsAcceleration(bax2);
        assertEquals(bax1, bax2);
        assertEquals(calibrator.getEstimatedBiasFx(), bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());

        final var bay1 = calibrator.getEstimatedBiasFyAsAcceleration();
        final var bay2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasFyAsAcceleration(bay2);
        assertEquals(bay1, bay2);
        assertEquals(calibrator.getEstimatedBiasFy(), bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());

        final var baz1 = calibrator.getEstimatedBiasFzAsAcceleration();
        final var baz2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasFzAsAcceleration(baz2);
        assertEquals(baz1, baz2);
        assertEquals(calibrator.getEstimatedBiasFz(), baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());

        final var bTriad1 = calibrator.getEstimatedBiasAsTriad();
        assertEquals(bTriad1.getValueX(), calibrator.getEstimatedBiasFx(), 0.0);
        assertEquals(bTriad1.getValueY(), calibrator.getEstimatedBiasFy(), 0.0);
        assertEquals(bTriad1.getValueZ(), calibrator.getEstimatedBiasFz(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bTriad1.getUnit());
        final var bTriad2 = new AccelerationTriad();
        calibrator.getEstimatedBiasAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);

        assertEquals(ma.getElementAt(0, 0), calibrator.getEstimatedSx(), 0.0);
        assertEquals(ma.getElementAt(1, 1), calibrator.getEstimatedSy(), 0.0);
        assertEquals(ma.getElementAt(2, 2), calibrator.getEstimatedSz(), 0.0);
        assertEquals(ma.getElementAt(0, 1), calibrator.getEstimatedMxy(), 0.0);
        assertEquals(ma.getElementAt(0, 2), calibrator.getEstimatedMxz(), 0.0);
        assertEquals(ma.getElementAt(1, 0), calibrator.getEstimatedMyx(), 0.0);
        assertEquals(ma.getElementAt(1, 2), calibrator.getEstimatedMyz(), 0.0);
        assertEquals(ma.getElementAt(2, 0), calibrator.getEstimatedMzx(), 0.0);
        assertEquals(ma.getElementAt(2, 1), calibrator.getEstimatedMzy(), 0.0);

        assertCovariance(calibrator);
    }

    private static void assertCovariance(final KnownFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        assertNotNull(calibrator.getEstimatedBiasFxVariance());
        assertNotNull(calibrator.getEstimatedBiasFxStandardDeviation());
        final var stdBx1 = calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration();
        assertNotNull(stdBx1);
        final var stdBx2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertTrue(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(stdBx2));
        assertEquals(stdBx1, stdBx2);

        assertNotNull(calibrator.getEstimatedBiasFyVariance());
        assertNotNull(calibrator.getEstimatedBiasFyStandardDeviation());
        final var stdBy1 = calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration();
        assertNotNull(stdBy1);
        final var stdBy2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertTrue(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(stdBy2));
        assertEquals(stdBy1, stdBy2);

        assertNotNull(calibrator.getEstimatedBiasFzVariance());
        assertNotNull(calibrator.getEstimatedBiasFzStandardDeviation());
        final var stdBz1 = calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration();
        assertNotNull(stdBz1);
        final var stdBz2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertTrue(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(stdBz2));
        assertEquals(stdBz1, stdBz2);

        final var std1 = calibrator.getEstimatedBiasStandardDeviation();
        assertEquals(calibrator.getEstimatedBiasFxStandardDeviation(), std1.getValueX(), 0.0);
        assertEquals(calibrator.getEstimatedBiasFyStandardDeviation(), std1.getValueY(), 0.0);
        assertEquals(calibrator.getEstimatedBiasFzStandardDeviation(), std1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final var std2 = new AccelerationTriad();
        calibrator.getEstimatedBiasStandardDeviation(std2);

        final var avgStd = (calibrator.getEstimatedBiasFxStandardDeviation()
                + calibrator.getEstimatedBiasFyStandardDeviation()
                + calibrator.getEstimatedBiasFzStandardDeviation()) / 3.0;
        assertEquals(avgStd, calibrator.getEstimatedBiasStandardDeviationAverage(), 0.0);
        final var avg1 = calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration();
        assertEquals(avgStd, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final var avg2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(avg2);
        assertEquals(avg1, avg2);

        assertEquals(std1.getNorm(), calibrator.getEstimatedBiasStandardDeviationNorm(), ABSOLUTE_ERROR);
        final var norm1 = calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration();
        assertEquals(std1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        final var norm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(norm2);
        assertEquals(norm1, norm2);
    }

    private static void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(12, covariance.getRows());
        assertEquals(12, covariance.getColumns());

        for (var j = 0; j < 12; j++) {
            final var colIsZero = j == 8 || j == 10 || j == 11;
            for (var i = 0; i < 12; i++) {
                final var rowIsZero = i == 8 || i == 10 || i == 11;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(12, covariance.getRows());
        assertEquals(12, covariance.getColumns());

        for (var i = 0; i < 12; i++) {
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
