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
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class PROSACRobustKnownPositionAccelerometerCalibratorTest implements 
        RobustKnownPositionAccelerometerCalibratorListener {

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

    private static final int MEASUREMENT_NUMBER = 1000;
    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final int OUTLIER_PERCENTAGE = 10;

    private static final double THRESHOLD = 1e-10;
    private static final double LARGE_THRESHOLD = 1e-2;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final double OUTLIER_ERROR_FACTOR = 100.0;

    private static final int TIMES = 100;

    private int calibrateStart;
    private int calibrateEnd;
    private int calibrateNextIteration;
    private int calibrateProgressChange;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS, 
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS, 
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL, 
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(measurements);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(true);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS, 
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1]));
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(m2));
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(m1,
                ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(m2,
                ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(ba,
                m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(ba,
                m4));
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

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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

        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                true);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertEquals(calibrator.getInitialMa(), ma1);
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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

        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                true, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements, bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, new double[1]));
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements, bias,
                this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, new double[1], this));
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                true, bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, new double[1]));
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                true, bias, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, new double[1], this));
    }

    @Test
    void testConstructor17() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements, ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, m2));
    }

    @Test
    void testConstructor18() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements, ba,
                this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, m2, this));
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                true, ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, m2));
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                true, ba, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, m2, this));
    }

    @Test
    void testConstructor21() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements, ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx,  calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, ba, m4));
    }

    @Test
    void testConstructor22() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements, ba, ma,
                this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, m1, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, m2, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, ba, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, ba, m4, this));
    }

    @Test
    void testConstructor23() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                true, ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, ba, m4));
    }

    @Test
    void testConstructor24() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(ecefPosition, measurements,
                true, ba, ma, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, m1, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, m2, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, ba, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                ecefPosition, measurements, true, ba, m4, this));
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

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor26() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor27() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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

        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                true);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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

        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                true, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor30() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements, bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, new double[1]));
    }

    @Test
    void testConstructor31() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements, bias,
                this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, new double[1], this));
    }

    @Test
    void testConstructor32() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                true, bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, new double[1]));
    }

    @Test
    void testConstructor33() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                true, bias, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, new double[1], this));
    }

    @Test
    void testConstructor34() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements, ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, m2));
    }

    @Test
    void testConstructor35() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements, ba,
                this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, m2, this));
    }

    @Test
    void testConstructor36() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                true, ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, m2));
    }

    @Test
    void testConstructor37() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                true, ba, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, m2, this));
    }

    @Test
    void testConstructor38() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements, ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, ba, m4));
    }

    @Test
    void testConstructor39() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements, ba, ma,
                this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, m1, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, m2, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, ba, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, ba, m4, this));
    }

    @Test
    void testConstructor40() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                true, ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, ba, m4));
    }

    @Test
    void testConstructor41() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(nedPosition, measurements,
                true, ba, ma, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, m1, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, m2, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, ba, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, ba, m4, this));
    }

    @Test
    void testConstructor42() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements));
    }

    @Test
    void testConstructor43() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, this));
    }

    @Test
    void testConstructor44() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, true);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, true));
    }

    @Test
    void testConstructor45() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, true, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, true, this));
    }

    @Test
    void testConstructor46() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, new double[1]));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, bias));
    }

    @Test
    void testConstructor47() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, bias, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, new double[1], this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, bias, this));
    }

    @Test
    void testConstructor48() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, true, bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, new double[1]));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, true, bias));
    }

    @Test
    void testConstructor49() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, true, bias, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, new double[1], this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, true, bias, this));
    }

    @Test
    void testConstructor50() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, m2));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, ba));
    }

    @Test
    void testConstructor51() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, ba, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, m2, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, ba, this));
    }

    @Test
    void testConstructor52() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, true, ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, m2));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, true, ba));
    }

    @Test
    void testConstructor53() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, true, ba, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, m2, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, true, ba, this));
    }

    @Test
    void testConstructor54() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, ba, m4));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, ba, ma));
    }

    @Test
    void testConstructor55() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, ba, ma, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, m1, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, m2, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, ba, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, ba, m4, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, ba, ma, this));
    }

    @Test
    void testConstructor56() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, true, ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, ba, m4));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, true, ba, ma));
    }

    @Test
    void testConstructor57() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, ecefPosition,
                measurements, true, ba, ma, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, m1, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, m2, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, ba, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, ecefPosition, measurements, true, ba, m4, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], ecefPosition, measurements, true, ba, ma, this));
    }

    @Test
    void testConstructor58() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements));
    }

    @Test
    void testConstructor59() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, this));
    }

    @Test
    void testConstructor60() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, true);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, true));
    }

    @Test
    void testConstructor61() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, true, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, true, this));
    }

    @Test
    void testConstructor62() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, new double[1]));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, bias));
    }

    @Test
    void testConstructor63() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, bias, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, new double[1], this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, bias, this));
    }

    @Test
    void testConstructor64() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, true, bias);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, new double[1]));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, true, bias));
    }

    @Test
    void testConstructor65() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, true, bias, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                nedPosition, measurements, true, new double[1], this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, true, bias, this));
    }

    @Test
    void testConstructor66() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, m2));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, ba));
    }

    @Test
    void testConstructor67() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, ba, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, m2, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, ba, this));
    }

    @Test
    void testConstructor68() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, true, ba);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, m2));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, true, ba));
    }

    @Test
    void testConstructor69() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, true, ba, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
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
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        final var ma1 = new Matrix(3, 3);
        assertEquals(ma1, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, m2, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, true, ba, this));
    }

    @Test
    void testConstructor70() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy,calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, ba, m4));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, ba, ma));
    }

    @Test
    void testConstructor71() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, ba, ma, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, m1, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, m2, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, ba, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, ba, m4, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, ba, ma, this));
    }

    @Test
    void testConstructor72() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                measurements, true, ba, ma);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, ba, m4));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, true, ba, ma));
    }

    @Test
    void testConstructor73() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var bias = ba.getBuffer();

        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var qualityScores = new double[13];

        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition, 
                measurements, true, ba, ma, this);

        // check default values
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(bx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(by, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bz, calibrator.getInitialBiasZ(), 0.0);
        var acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(bx, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(by, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(bz, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(by, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        final var bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        final var biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(ma, calibrator.getInitialMa());
        final var ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations(),
                0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
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
        assertEquals(RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, m1, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, m2, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, ba, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                qualityScores, nedPosition, measurements, true, ba, m4, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustKnownPositionAccelerometerCalibrator(
                new double[1], nedPosition, measurements, true, ba, ma, this));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(),
                0.0);

        // set new value
        calibrator.setThreshold(0.1);

        // check
        assertEquals(0.1, calibrator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setThreshold(0.0));
    }

    @Test
    void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());

        // set new value
        calibrator.setComputeAndKeepInliersEnabled(true);

        // check
        assertTrue(calibrator.isComputeAndKeepInliersEnabled());
    }

    @Test
    void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertFalse(calibrator.isComputeAndKeepResiduals());

        // set new value
        calibrator.setComputeAndKeepResidualsEnabled(true);

        // check
        assertTrue(calibrator.isComputeAndKeepResiduals());
    }

    @Test
    void testGetSetInitialBiasX() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);

        // set new value
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);

        calibrator.setInitialBiasX(biasX);

        // check
        assertEquals(biasX, calibrator.getInitialBiasX(), 0.0);
    }

    @Test
    void testGetSetInitialBiasY() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);

        // set new value
        final var ba = generateBa();
        final var biasY = ba.getElementAtIndex(1);

        calibrator.setInitialBiasY(biasY);

        // check
        assertEquals(biasY, calibrator.getInitialBiasY(), 0.0);
    }

    @Test
    void testGetSetInitialBiasZ() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new value
        final var ba = generateBa();
        final var biasZ = ba.getElementAtIndex(2);

        calibrator.setInitialBiasZ(biasZ);

        // check
        assertEquals(biasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testGetSetInitialBiasXAsAcceleration() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        final var biasX1 = calibrator.getInitialBiasXAsAcceleration();

        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasX1.getUnit());

        // set new value
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);

        final var biasX2 = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        final var biasY1 = calibrator.getInitialBiasYAsAcceleration();

        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasY1.getUnit());

        // set new value
        final var ba = generateBa();
        final var biasY = ba.getElementAtIndex(1);

        final var biasY2 = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        final var biasZ1 = calibrator.getInitialBiasZAsAcceleration();

        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasZ1.getUnit());

        // set new value
        final var ba = generateBa();
        final var biasZ = ba.getElementAtIndex(2);

        final var biasZ2 = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new value
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        calibrator.setInitialBias(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(biasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testSetInitialBias2() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new value
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var bax = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setInitialBias(bax, bay, baz);

        // check
        assertEquals(biasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(biasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
    void testGetSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
    void testGetSetInitialCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
    void testGetSetInitialBias() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        final var bias1 = calibrator.getInitialBias();
        assertArrayEquals(new double[3], bias1, 0.0);

        // set new value
        final var bias2 = generateBa().getBuffer();
        calibrator.setInitialBias(bias2);

        // check
        final var bias3 = calibrator.getInitialBias();
        final var bias4 = new double[3];
        calibrator.getInitialBias(bias4);

        assertArrayEquals(bias2, bias3, 0.0);
        assertArrayEquals(bias2, bias4, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(new double[1]));
    }

    @Test
    void testGetSetInitialBiasAsMatrix() throws LockedException, WrongSizeException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        final var bias1 = calibrator.getInitialBiasAsMatrix();
        assertArrayEquals(new double[3], bias1.getBuffer(), 0.0);

        // set new values
        final var bias2 = generateBa();
        calibrator.setInitialBias(bias2);

        // check
        final var bias3 = calibrator.getInitialBiasAsMatrix();
        final var bias4 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bias4);

        assertEquals(bias2, bias3);
        assertEquals(bias2, bias4);

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
    void testGetSetInitialMa() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        final var ma1 = calibrator.getInitialMa();
        assertEquals(new Matrix(3, 3), ma1);

        // set new value
        final var ma2 = generateMaGeneral();
        calibrator.setInitialMa(ma2);

        final var initialSx = ma2.getElementAt(0, 0);
        final var initialSy = ma2.getElementAt(1, 1);
        final var initialSz = ma2.getElementAt(2, 2);
        final var initialMxy = ma2.getElementAt(0, 1);
        final var initialMxz = ma2.getElementAt(0, 2);
        final var initialMyx = ma2.getElementAt(1, 0);
        final var initialMyz = ma2.getElementAt(1, 2);
        final var initialMzx = ma2.getElementAt(2, 0);
        final var initialMzy = ma2.getElementAt(2, 1);

        // check
        final var ma3 = calibrator.getInitialMa();
        final var ma4 = new Matrix(3, 3);
        calibrator.getInitialMa(ma4);

        assertEquals(ma2, ma3);
        assertEquals(ma2, ma3);

        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);

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
    void testGetSetEcefPosition() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition1 = new NEDPosition(latitude, longitude, height);

        calibrator.setPosition(nedPosition1);

        // check
        final var nedPosition2 = calibrator.getNedPosition();
        final var nedPosition3 = new NEDPosition();
        calibrator.getNedPosition(nedPosition3);

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedPosition1.equals(nedPosition3, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testIsReady() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check
        assertFalse(calibrator.isReady());

        // set empty measurements
        final var measurements1 = Collections.<StandardDeviationBodyKinematics>emptyList();
        calibrator.setMeasurements(measurements1);

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements for general case
        calibrator.setCommonAxisUsed(false);

        final var measurements2 = new ArrayList<StandardDeviationBodyKinematics>();
        for (var i = 0; i < RobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL; i++) {
            measurements2.add(new StandardDeviationBodyKinematics());
        }
        calibrator.setMeasurements(measurements2);

        // check
        assertFalse(calibrator.isReady());

        // set position
        final var position = new ECEFPosition();
        calibrator.setPosition(position);

        assertFalse(calibrator.isReady());

        // set quality scores with different length
        var qualityScores = new double[14];
        calibrator.setQualityScores(qualityScores);

        assertFalse(calibrator.isReady());

        // set quality scores with equal length
        qualityScores = new double[13];
        calibrator.setQualityScores(qualityScores);

        assertTrue(calibrator.isReady());

        // set enough measurements for common axis case
        measurements2.clear();
        for (var i = 0; i < KnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS; i++) {
            measurements2.add(new StandardDeviationBodyKinematics());
        }
        calibrator.setMeasurements(measurements2);

        // check
        assertFalse(calibrator.isReady());

        calibrator.setCommonAxisUsed(true);

        assertFalse(calibrator.isReady());

        // set quality scores with equal length
        qualityScores = new double[10];
        calibrator.setQualityScores(qualityScores);

        assertTrue(calibrator.isReady());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertEquals(0.05f, calibrator.getProgressDelta(), 0.0);

        // set new value
        calibrator.setProgressDelta(0.01f);

        // check
        assertEquals(0.01f, calibrator.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertEquals(0.99, calibrator.getConfidence(), 0.0);

        // set new value
        calibrator.setConfidence(0.5);

        // check
        assertEquals(0.5, calibrator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertEquals(5000, calibrator.getMaxIterations());

        // set new value
        calibrator.setMaxIterations(100);

        assertEquals(100, calibrator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        final var qualityScores = new double[
                PROSACRobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL];
        calibrator.setQualityScores(qualityScores);

        // check
        assertSame(calibrator.getQualityScores(), qualityScores);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setQualityScores(new double[12]));
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator();

        // check default value
        assertEquals(PROSACRobustKnownPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());

        // set new value
        calibrator.setPreliminarySubsetSize(14);

        // check
        assertEquals(14, calibrator.getPreliminarySubsetSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setPreliminarySubsetSize(9));
    }

    @Test
    void testCalibrateGeneralNoNoiseInlier() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException {
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

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final var errorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
                    0.0, 0.0, accelQuantLevel, gyroQuantLevel);
            
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, specificForceStandardDeviation);

            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsOutlier, random);
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsInlier, random);
                    error = 0.0;
                }

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);

                qualityScores[i] = 1.0 / (1.0 + error);
            }

            final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                    measurements, false, ba, ma, this);
            calibrator.setThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator, true);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateCommonAxisNoNoiseInlier() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, CalibrationException, NotReadyException {
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

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final var errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0, 0.0,
                    accelQuantLevel, gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, specificForceStandardDeviation);

            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsOutlier, random);
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsInlier, random);
                    error = 0.0;
                }

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);

                qualityScores[i] = 1.0 / (1.0 + error);
            }

            final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                    measurements, true, ba, ma, this);
            calibrator.setThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator, true);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralWithInlierNoise() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException {
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

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final var errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, specificForceStandardDeviation);

            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var qualityScores = new double[LARGE_MEASUREMENT_NUMBER];
            double error;
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsOutlier, random);
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsInlier, random);
                    error = 0.0;
                }

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);

                qualityScores[i] = 1.0 / (1.0 + error);
            }

            final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                    measurements, false, this);
            calibrator.setThreshold(LARGE_THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

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

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator, true);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateCommonAxisWithInlierNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, CalibrationException, NotReadyException {
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

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final var errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, specificForceStandardDeviation);

            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var qualityScores = new double[LARGE_MEASUREMENT_NUMBER];
            double error;
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsOutlier, random);
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsInlier, random);
                    error = 0.0;
                }

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);

                qualityScores[i] = 1.0 / (1.0 + error);
            }

            final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                    measurements, true, this);
            calibrator.setThreshold(LARGE_THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

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

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator, true);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralNoRefinement() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException {
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

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final var errorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
                    0.0, 0.0, accelQuantLevel, gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, specificForceStandardDeviation);

            final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsOutlier, random);
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                            errorsInlier, random);
                    error = 0.0;
                }

                final var measurement = new StandardDeviationBodyKinematics(measuredKinematics,
                        specificForceStandardDeviation, angularRateStandardDeviation);
                measurements.add(measurement);

                qualityScores[i] = 1.0 / (1.0 + error);
            }

            final var calibrator = new PROSACRobustKnownPositionAccelerometerCalibrator(qualityScores, nedPosition,
                    measurements, false, ba, ma, this);
            calibrator.setThreshold(THRESHOLD);
            calibrator.setResultRefined(false);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator, false);

            assertNotNull(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() >= 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustKnownPositionAccelerometerCalibrator calibrator) {
        checkLocked((PROSACRobustKnownPositionAccelerometerCalibrator) calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final RobustKnownPositionAccelerometerCalibrator calibrator) {
        checkLocked((PROSACRobustKnownPositionAccelerometerCalibrator) calibrator);
        calibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownPositionAccelerometerCalibrator calibrator, final int iteration) {
        checkLocked((PROSACRobustKnownPositionAccelerometerCalibrator) calibrator);
        calibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownPositionAccelerometerCalibrator calibrator, final float progress) {
        checkLocked((PROSACRobustKnownPositionAccelerometerCalibrator) calibrator);
        calibrateProgressChange++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
        calibrateNextIteration = 0;
        calibrateProgressChange = 0;
    }

    private void checkLocked(final PROSACRobustKnownPositionAccelerometerCalibrator calibrator) {
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
        assertThrows(LockedException.class, () -> calibrator.setInitialMa(null));
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, () -> calibrator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> calibrator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> calibrator.setMaxIterations(500));
        assertThrows(LockedException.class, () -> calibrator.setResultRefined(true));
        assertThrows(LockedException.class, () -> calibrator.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> calibrator.setPreliminarySubsetSize(5));
        assertThrows(LockedException.class, calibrator::calibrate);
        assertThrows(LockedException.class, () -> calibrator.setThreshold(0.1));
        assertThrows(LockedException.class, () -> calibrator.setQualityScores(null));
        assertThrows(LockedException.class, () -> calibrator.setComputeAndKeepInliersEnabled(true));
        assertThrows(LockedException.class, () -> calibrator.setComputeAndKeepResidualsEnabled(true));
    }

    private static void assertEstimatedResult(
            final Matrix ba, final Matrix ma, final PROSACRobustKnownPositionAccelerometerCalibrator calibrator,
            final boolean checkCovariance) throws WrongSizeException {

        final var estimatedBiases = calibrator.getEstimatedBiases();
        assertArrayEquals(estimatedBiases, ba.getBuffer(), 0.0);

        final var estimatedBiases2 = new double[3];
        calibrator.getEstimatedBiases(estimatedBiases2);
        assertArrayEquals(estimatedBiases2, estimatedBiases, 0.0);

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
        assertEquals(bax1.getValue().doubleValue(), calibrator.getEstimatedBiasFx(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());

        final var bay1 = calibrator.getEstimatedBiasFyAsAcceleration();
        final var bay2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasFyAsAcceleration(bay2);
        assertEquals(bay1, bay2);
        assertEquals(bay1.getValue().doubleValue(), calibrator.getEstimatedBiasFy(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());

        final var baz1 = calibrator.getEstimatedBiasFzAsAcceleration();
        final var baz2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasFzAsAcceleration(baz2);
        assertEquals(baz1, baz2);
        assertEquals(baz1.getValue().doubleValue(), calibrator.getEstimatedBiasFz(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());

        assertEquals(ma.getElementAt(0, 0), calibrator.getEstimatedSx(), 0.0);
        assertEquals(ma.getElementAt(1, 1), calibrator.getEstimatedSy(), 0.0);
        assertEquals(ma.getElementAt(2, 2), calibrator.getEstimatedSz(), 0.0);
        assertEquals(ma.getElementAt(0, 1), calibrator.getEstimatedMxy(), 0.0);
        assertEquals(ma.getElementAt(0, 2), calibrator.getEstimatedMxz(), 0.0);
        assertEquals(ma.getElementAt(1, 0), calibrator.getEstimatedMyx(), 0.0);
        assertEquals(ma.getElementAt(1, 2), calibrator.getEstimatedMyz(), 0.0);
        assertEquals(ma.getElementAt(2, 0), calibrator.getEstimatedMzx(), 0.0);
        assertEquals(ma.getElementAt(2, 1), calibrator.getEstimatedMzy(), 0.0);

        if (checkCovariance) {
            assertCovariance(calibrator);
        }
    }

    private static void assertCovariance(final PROSACRobustKnownPositionAccelerometerCalibrator calibrator) {
        assertNotNull(calibrator.getEstimatedBiasFxVariance());

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
        assertEquals(std1.getValueX(), calibrator.getEstimatedBiasFxStandardDeviation(), 0.0);
        assertEquals(std1.getValueY(), calibrator.getEstimatedBiasFyStandardDeviation(), 0.0);
        assertEquals(std1.getValueZ(), calibrator.getEstimatedBiasFzStandardDeviation(), 0.0);
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
